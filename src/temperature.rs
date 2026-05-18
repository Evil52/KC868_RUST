//! Pt100 sampling task.
//!
//! Reads the MAX31865 every `SAMPLE_PERIOD_MS`, converts to °C, and signals
//! the latest value to the MQTT task. Faults are logged but do not abort
//! the task — sensor cables get yanked, that's normal in the field.

use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiDevice;
use log::{info, warn};

use crate::bsp::max31865::{RREF_OHMS, R0_OHMS, SAMPLE_PERIOD_MS};
use crate::max31865::{Error as RtdError, Max31865};
use crate::mqtt::TEMPERATURE;
use crate::pt100::resistance_to_celsius;

#[embassy_executor::task]
pub async fn temperature_task(mut sensor: Max31865<crate::SpiDev>) {
    if let Err(e) = sensor.init().await {
        warn!("max31865: init failed: {:?}", e);
    }

    loop {
        match sensor.read_resistance(RREF_OHMS).await {
            Ok(r) => {
                let t = resistance_to_celsius(r, R0_OHMS);
                info!("pt100: R = {:.2} Ω, T = {:.2} °C", r, t);
                TEMPERATURE.signal(t);
            }
            Err(RtdError::Fault(f)) => warn!("pt100: fault 0x{:02X}", f),
            Err(RtdError::Spi(e))   => warn!("pt100: spi error {:?}", e),
        }
        Timer::after(Duration::from_millis(SAMPLE_PERIOD_MS)).await;
    }
}

// Force-link this trait so older toolchains don't drop the impl.
#[allow(dead_code)]
fn _assert_spi<S: SpiDevice<u8>>() {}
