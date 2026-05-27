//! Relay controller.
//!
//! Signal chain (from schematic): ESP32 → PCF8574 (U3) → 74HCT14 inverter →
//! ULN2003A Darlington → relay coil. Net inversion count is **odd**, so the
//! bus logic is **active-low**: write a `0` bit to energise the relay,
//! `1` to release it. Boot value is `0xFF` (all OFF) — anything else
//! switches mains loads on for the boot duration.
//!
//! The driver runs as its own task and is fed RelayCommand values through
//! an embassy Channel; other tasks (MQTT, control loop, button handler)
//! talk to it without ever touching the I2C bus directly.

use core::num::NonZeroU8;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c;
use log::{info, warn};

use crate::bsp::relay::{BITS, COUNT, IDLE_PORT_VALUE};
use crate::pcf8574::Pcf8574;
use crate::safety;

/// Per-channel command debounce. Matches the working KinCony reference
/// firmware; throttles rapid-fire MQTT commands (e.g., a UI double-click
/// or a script in a tight loop) to protect the mechanical relay from
/// premature wear and stops audible chatter. AllOff / SetMask bypass
/// the per-channel gate — they're treated as bulk overrides.
const COMMAND_DEBOUNCE: Duration = Duration::from_millis(150);

/// Public, lock-free view of the current ON-mask (bit per relay, 1 = ON).
/// Updated by `apply()` after every successful write. Read by display
/// and diagnostics tasks.
static ON_MASK: AtomicU8 = AtomicU8::new(0);

/// Pulsed every time `apply()` succeeds — the MQTT publisher uses this
/// to know when to re-emit per-relay retained state.
pub static STATE_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[inline]
pub fn current_on_mask() -> u8 {
    ON_MASK.load(Ordering::Acquire)
}

/// Helper: build a `NonZeroU8` at const-time. The `panic!` in the
/// `None` arm fires only during **const evaluation** — if a caller
/// ever writes `nz(0)` the build fails outright. With the literal
/// arguments (`3`, `6`) we use, the arm is dead-code-eliminated and
/// the compiled binary contains no panic path. Same idiom as
/// `NonZeroU8::new(N).unwrap()` in core/std.
const fn nz(n: u8) -> NonZeroU8 {
    match NonZeroU8::new(n) {
        Some(v) => v,
        None => panic!("nz(): expected non-zero argument"), // compile-time only
    }
}

/// Number of init attempts before giving up. Cold-boot I²C state on the
/// PCF8574 can take a couple of retries to settle (the 74HCT14 + ULN2003A
/// inputs upstream may briefly hold SDA low).
const INIT_ATTEMPTS: u8 = 5;
/// Retry count for runtime writes — applied to every `apply()`, but the
/// `AllOff` path retries longer (see `APPLY_ATTEMPTS_ALLOFF`). Stored
/// as `NonZeroU8` so `write_with_retry` cannot be called with zero
/// attempts and the "no error to return" panic-path doesn't exist.
const APPLY_ATTEMPTS:        NonZeroU8 = nz(3);
const APPLY_ATTEMPTS_ALLOFF: NonZeroU8 = nz(6);

/// Error returned when `Relays::new` cannot bring the expander up.
#[derive(Debug)]
pub struct InitError;

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // SetMask/AllOff are part of the public API for future callers
pub enum RelayCommand {
    Set { index: u8, on: bool },
    SetMask(u8),
    AllOff,
}

pub type RelayChannel = Channel<CriticalSectionRawMutex, RelayCommand, 16>;
pub type RelayTx = Sender<'static, CriticalSectionRawMutex, RelayCommand, 16>;

pub struct Relays<BUS> {
    expander: Pcf8574<BUS>,
    /// Per-channel debounce timestamps. `None` until the first command
    /// reaches that channel — at which point we record the success time
    /// and reject anything arriving sooner than `COMMAND_DEBOUNCE`.
    last_change: [Option<Instant>; COUNT],
}

impl<BUS, E> Relays<BUS>
where
    BUS: I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Bring the expander up. Returns `Err(InitError)` after
    /// `INIT_ATTEMPTS` consecutive failures — caller must treat that as
    /// fatal: if I²C is dead at boot, the chip is in an unknown state
    /// (relays might be stuck ON from the previous cycle, or a foreign
    /// device might be on the bus) and we cannot honour any further
    /// commands safely.
    pub async fn new(bus: BUS, addr: u8) -> Result<Self, InitError> {
        let mut expander = Pcf8574::new(bus, addr, IDLE_PORT_VALUE);
        for attempt in 1..=INIT_ATTEMPTS {
            match expander.commit().await {
                Ok(()) => {
                    info!("relay expander @0x{:02X} ready (attempt {})", addr, attempt);
                    return Ok(Self {
                        expander,
                        last_change: [None; COUNT],
                    });
                }
                Err(e) if attempt < INIT_ATTEMPTS => {
                    warn!("relay init attempt {} failed: {:?} — retrying", attempt, e);
                    Timer::after(Duration::from_millis(20 * attempt as u64)).await;
                }
                Err(e) => warn!("relay init giving up after {} attempts: {:?}", attempt, e),
            }
        }
        Err(InitError)
    }

    /// Apply one command. Respects the global safety interlock: when the
    /// interlock is engaged, every command except `AllOff` is dropped
    /// (and we proactively re-issue `AllOff` to make sure the bus state
    /// matches the safety contract).
    pub async fn apply(&mut self, cmd: RelayCommand) {
        if safety::is_locked() {
            match cmd {
                RelayCommand::AllOff => {} // pass through
                _ => {
                    warn!("relay: ignoring {:?} — safety interlock engaged", cmd);
                    let _ = self.write_with_retry(IDLE_PORT_VALUE, APPLY_ATTEMPTS_ALLOFF).await;
                    return;
                }
            }
        }

        let target_port = match cmd {
            RelayCommand::Set { index, on } => {
                let idx_usize = index as usize;
                if idx_usize >= COUNT {
                    warn!("relay: ignoring index {} (max {})", index, COUNT - 1);
                    return;
                }
                // Per-channel debounce — reject if the previous successful
                // command on this channel was less than COMMAND_DEBOUNCE ago.
                if let Some(prev) = self.last_change[idx_usize] {
                    let elapsed = Instant::now()
                        .checked_duration_since(prev)
                        .unwrap_or(Duration::MAX);
                    if elapsed < COMMAND_DEBOUNCE {
                        warn!(
                            "relay {}: debounce — {} ms since last (need {} ms)",
                            index, elapsed.as_millis(), COMMAND_DEBOUNCE.as_millis(),
                        );
                        return;
                    }
                }
                let bit  = BITS[idx_usize];
                let mask = 1u8 << bit;
                // Active-low: clear the bit to energise.
                if on { self.expander.shadow() & !mask } else { self.expander.shadow() | mask }
            }
            RelayCommand::SetMask(mask) => {
                // `mask`: bit per relay, "1 = ON". Convert to active-low
                // and force the unused upper bits to 1 (OFF). Bulk override
                // bypasses per-channel debounce (single I²C transaction).
                let active_low = !mask & 0x3F;
                active_low | !0x3F
            }
            RelayCommand::AllOff => IDLE_PORT_VALUE,
        };

        let attempts = match cmd {
            RelayCommand::AllOff => APPLY_ATTEMPTS_ALLOFF,
            _                    => APPLY_ATTEMPTS,
        };

        match self.write_with_retry(target_port, attempts).await {
            Ok(()) => {
                let on_bits = (!self.expander.shadow()) & 0x3F;
                ON_MASK.store(on_bits, Ordering::Release);
                STATE_CHANGED.signal(());
                // Record the debounce timestamp only for the channel
                // actually changed by this command (Set). Bulk commands
                // don't update per-channel timestamps.
                if let RelayCommand::Set { index, .. } = cmd {
                    if let Some(slot) = self.last_change.get_mut(index as usize) {
                        *slot = Some(Instant::now());
                    }
                }
                info!("relay ON-mask = 0b{:06b}", on_bits);
            }
            Err(e) => {
                warn!("relay write failed after {} attempts: {:?}", attempts, e);
                if matches!(cmd, RelayCommand::AllOff) {
                    // Cannot prove the relays are off. Hardware reset is
                    // the only safe path — PCF8574 powers up tri-stated
                    // (port reads as 0xFF), which the active-low chain
                    // interprets as all-OFF.
                    log::error!("relay: AllOff failed irrecoverably → software reset");
                    esp_hal::reset::software_reset();
                }
            }
        }
    }

    async fn write_with_retry(&mut self, port_byte: u8, attempts: NonZeroU8) -> Result<(), E> {
        let total = attempts.get();

        // First attempt unconditionally — guarantees `last_err` is
        // initialised without an Option dance, removing the panic path
        // from the "0 attempts" edge case.
        let mut last_err = match self.expander.write_port(port_byte).await {
            Ok(()) => return Ok(()),
            Err(crate::pcf8574::Error::Bus(e)) => e,
        };

        for attempt in 2..=total {
            Timer::after(Duration::from_millis(10 * (attempt - 1) as u64)).await;
            match self.expander.write_port(port_byte).await {
                Ok(()) => return Ok(()),
                Err(crate::pcf8574::Error::Bus(e)) => last_err = e,
            }
        }
        Err(last_err)
    }
}

/// Long-running task: consume commands from the channel and apply them.
#[embassy_executor::task]
pub async fn relay_task(
    mut relays: Relays<crate::I2cBus>,
    rx: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, RelayCommand, 16>,
) {
    loop {
        let cmd = rx.receive().await;
        relays.apply(cmd).await;
    }
}
