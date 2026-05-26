//! Pt100 sampling task.
//!
//! Reads the MAX31865 every `SAMPLE_PERIOD_MS`, converts to °C, and signals
//! the latest value to the MQTT task. Faults are tolerated — sensor cables
//! get yanked in the field — but with bounded noise on the log:
//!   * if `init()` fails we re-attempt at 60-second intervals instead of
//!     spamming the bus and the log every second
//!   * a steady stream of read errors triggers one warn per 30 s, not per
//!     second
//!   * conversion failures (out-of-range RTD, broken wire) downgrade the
//!     signalled value to "no sample" rather than poison MQTT with a bogus
//!     -242 °C

use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiDevice;
use log::{info, warn};

use crate::bsp::max31865::{RREF_OHMS, R0_OHMS, SAMPLE_PERIOD_MS};
use crate::max31865::{Error as RtdError, Max31865};
use crate::mqtt::TEMPERATURE;
use crate::pt100::resistance_to_celsius;

const INIT_RETRY:  Duration = Duration::from_secs(60);
const ERROR_LOG_PERIOD_TICKS: u32 = 30;

#[embassy_executor::task]
pub async fn temperature_task(mut sensor: Max31865<crate::SpiDev>) {
    // -------- init with slow retry --------------------------------------
    loop {
        match sensor.init().await {
            Ok(()) => { info!("max31865: ready"); break; }
            Err(e) => {
                warn!("max31865: init failed ({:?}) — retry in {} s",
                      e, INIT_RETRY.as_secs());
                Timer::after(INIT_RETRY).await;
            }
        }
    }

    // -------- sample loop ----------------------------------------------
    let mut consecutive_errors: u32 = 0;

    loop {
        match sensor.read_resistance(RREF_OHMS).await {
            Ok(r) => {
                if let Some(t) = resistance_to_celsius(r, R0_OHMS) {
                    if consecutive_errors > 0 {
                        info!("pt100: recovered after {} error(s)", consecutive_errors);
                        consecutive_errors = 0;
                    }
                    info!("pt100: R = {:.2} Ω, T = {:.2} °C", r, t);
                    TEMPERATURE.signal(t);
                } else {
                    consecutive_errors = consecutive_errors.saturating_add(1);
                    if consecutive_errors == 1 || consecutive_errors % ERROR_LOG_PERIOD_TICKS == 0 {
                        warn!("pt100: out-of-range conversion (R={:.2} Ω) — sensor wiring?", r);
                    }
                }
            }
            Err(e) => {
                consecutive_errors = consecutive_errors.saturating_add(1);
                if consecutive_errors == 1 || consecutive_errors % ERROR_LOG_PERIOD_TICKS == 0 {
                    match &e {
                        RtdError::Fault(f) => warn!("pt100: fault 0x{:02X} (x{})", f, consecutive_errors),
                        RtdError::Spi(se)  => warn!("pt100: spi error {:?} (x{})", se, consecutive_errors),
                    }
                }
            }
        }
        Timer::after(Duration::from_millis(SAMPLE_PERIOD_MS)).await;
    }
}

// Force-link this trait so older toolchains don't drop the impl.
#[allow(dead_code)]
fn _assert_spi<S: SpiDevice<u8>>() {}
