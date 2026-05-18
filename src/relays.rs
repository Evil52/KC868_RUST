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

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;
use log::{info, warn};

use crate::bsp::relay::{BITS, COUNT, IDLE_PORT_VALUE};
use crate::pcf8574::Pcf8574;

/// Number of init attempts before giving up. Cold-boot I²C state on the
/// PCF8574 can take a couple of retries to settle (the 74HCT14 + ULN2003A
/// inputs upstream may briefly hold SDA low).
const INIT_ATTEMPTS: u8 = 5;

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
}

impl<BUS, E> Relays<BUS>
where
    BUS: I2c<Error = E>,
    E: core::fmt::Debug,
{
    pub async fn new(bus: BUS, addr: u8) -> Self {
        // Start with every relay de-energised (active-low: 0xFF = all OFF).
        let mut expander = Pcf8574::new(bus, addr, IDLE_PORT_VALUE);
        for attempt in 1..=INIT_ATTEMPTS {
            match expander.commit().await {
                Ok(()) => {
                    info!("relay expander @0x{:02X} ready (attempt {})", addr, attempt);
                    break;
                }
                Err(e) if attempt < INIT_ATTEMPTS => {
                    warn!("relay init attempt {} failed: {:?} — retrying", attempt, e);
                    Timer::after(Duration::from_millis(20 * attempt as u64)).await;
                }
                Err(e) => warn!("relay init giving up after {} attempts: {:?}", attempt, e),
            }
        }
        Self { expander }
    }

    pub async fn apply(&mut self, cmd: RelayCommand) {
        let result = match cmd {
            RelayCommand::Set { index, on } => {
                if (index as usize) < COUNT {
                    let bit = BITS[index as usize];
                    // Active-low: pull the bit LOW to energise.
                    self.expander.set_bit(bit, !on).await
                } else {
                    warn!("ignoring relay index {} (max {})", index, COUNT - 1);
                    Ok(())
                }
            }
            RelayCommand::SetMask(mask) => {
                // mask: 1 bit per relay, MSB-aligned at COUNT, semantics
                // "1 = ON". Convert to active-low port byte: invert the
                // active bits and force the unused upper bits to 1 (OFF).
                let active_low = !mask & 0x3F;
                let port_byte  = active_low | !0x3F;
                self.expander.write_port(port_byte).await
            }
            RelayCommand::AllOff => self.expander.write_port(IDLE_PORT_VALUE).await,
        };

        if let Err(e) = result {
            warn!("relay write failed: {:?}", e);
        } else {
            // Log the ON-bits (semantic view, not the raw port byte).
            let on_bits = (!self.expander.shadow()) & 0x3F;
            info!("relay ON-mask = 0b{:06b}", on_bits);
        }
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
