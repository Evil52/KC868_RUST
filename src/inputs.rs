//! Opto-input task (U6 PCF8574 @ 0x22).
//!
//! Reads the 6 opto-isolated 12 V digital inputs every
//! `bsp::input::POLL_PERIOD_MS`, exposes the current state as an atomic
//! mask, and — if `bsp::input::ARM_SAFETY` is true — drives the global
//! safety interlock:
//!
//!   * Convention: with a fail-safe loop (12 V through a chain of NC
//!     contacts), all inputs energised ⇒ all PCF8574 bits LOW ⇒ a
//!     `0x00` reading on the lower 6 bits means SAFE.
//!   * Any bit going HIGH (loop broken: E-Stop pressed, door opened,
//!     wire cut) engages the interlock and emits `RelayCommand::AllOff`.
//!   * Once locked, the interlock requires an external reset
//!     (`kc868/safety/reset` MQTT command **and** all inputs back to
//!     SAFE) — no auto-clear.
//!
//! Debounce: a single alarm reading is NOT enough — mechanical contacts
//! and reed switches bounce for 5–50 ms. We require
//! `DEBOUNCE_REQUIRED` consecutive alarm reads before engaging the
//! interlock. With 100 ms polling that gives ~200 ms response time —
//! well inside typical industrial-safety expectations and immune to
//! single-tick glitches.

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_time::{Duration, Timer};
use log::{info, warn};

use crate::bsp::input;
use crate::pcf8574::Pcf8574;
use crate::relays::{RelayCommand, RelayTx};
use crate::safety;

/// Consecutive alarm reads required to commit to the interlock. 2 with
/// a 100 ms poll = 200 ms response; one spike will not lock.
const DEBOUNCE_REQUIRED: u8 = 2;

/// Public, lock-free view of the current input mask (bit per input,
/// 1 = energised / 12 V present). Updated every poll cycle.
static INPUT_MASK: AtomicU8 = AtomicU8::new(0);

#[inline]
pub fn current_mask() -> u8 {
    INPUT_MASK.load(Ordering::Acquire)
}

#[embassy_executor::task]
pub async fn input_task(bus: crate::I2cBus, addr: u8, relay_tx: RelayTx) {
    let mut chip = Pcf8574::new(bus, addr, 0xFF);

    // Drive the port HIGH so the open-drain output transistors are off
    // and the opto-collector pull-downs can sink individual bits LOW.
    // Cold-boot the chip is already HIGH, but a software_reset() could
    // have left the latch in any state — make it deterministic.
    if let Err(e) = chip.commit().await {
        warn!("inputs: init commit failed: {:?} — continuing", e);
    } else {
        info!("inputs: U6 @0x{:02X} configured for input", addr);
    }

    let mut last_logged: Option<u8> = None;
    let mut alarm_streak: u8 = 0;

    loop {
        match chip.read_port().await {
            Ok(raw) => {
                // Active bits = LOW reading on opto output (12 V at terminal).
                // Convert to "1 = energised" semantics for the rest of the app.
                let energised = (!raw) & input::MASK;
                INPUT_MASK.store(energised, Ordering::Release);

                if last_logged != Some(energised) {
                    info!("inputs: 0b{:06b}", energised);
                    last_logged = Some(energised);
                }

                if input::ARM_SAFETY {
                    // Fail-safe loop convention: all inputs must be
                    // energised for the controller to be in SAFE state.
                    let safe = energised == input::MASK;
                    if safe {
                        alarm_streak = 0;
                    } else {
                        // Count this as an alarm sample. Saturate so we
                        // don't roll back to 0 after long alarms.
                        alarm_streak = alarm_streak.saturating_add(1);
                        if alarm_streak >= DEBOUNCE_REQUIRED
                            && safety::lock("input opened")
                        {
                            relay_tx.send(RelayCommand::AllOff).await;
                        }
                    }
                }
            }
            Err(e) => warn!("inputs: read failed: {:?}", e),
        }

        Timer::after(Duration::from_millis(input::POLL_PERIOD_MS)).await;
    }
}

/// Try to clear the safety interlock — succeeds only if currently locked
/// AND all inputs are in the SAFE state. Called from the MQTT subscriber
/// when `kc868/safety/reset` arrives.
pub fn try_safety_reset() -> bool {
    if !safety::is_locked() {
        return true; // already clear
    }
    if !input::ARM_SAFETY {
        // No safety arming configured — reset is a no-op success.
        return safety::unlock("manual reset, safety not armed");
    }
    let energised = current_mask();
    if energised == input::MASK {
        safety::unlock("manual reset, all inputs safe")
    } else {
        warn!(
            "safety reset rejected: inputs not all safe (mask=0b{:06b}, expected 0b{:06b})",
            energised, input::MASK
        );
        false
    }
}
