#![no_std]
#![no_main]

//! KC868-A6 firmware — async embassy on ESP32.
//!
//! Tasks:
//!   * `wifi::connection_task` / `net_task` — WiFi + IP stack.
//!   * `mqtt_publisher_task`                — publishes status + temperature.
//!   * `mqtt_subscriber_task`               — receives relay commands.
//!   * `relay_task`                         — drives PCF8574 relay outputs.
//!   * `display_task`                       — SSD1306 OLED on the I²C bus.
//!
//! `max31865` / `temperature` / `pt100` are kept in-tree but **not spawned**:
//! the Pt100 frontend hardware is not populated yet. Wire it up, then
//! re-enable the SPI setup + `temperature_task` spawn below.

extern crate alloc;

mod bsp;
mod button;
mod display;
mod ha_discovery;
mod hw_watchdog;
mod inputs;
mod pcf8574;
mod relays;
mod max31865;
mod pt100;
mod safety;
mod temperature;
mod watchdog;
mod wifi;
mod mqtt;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, Output, Pull};
use esp_hal::rng::Rng;
use esp_hal::time::RateExtU32;
use esp_hal::timer::timg::TimerGroup;
use log::info;
use static_cell::StaticCell;

// ---------------------------------------------------------------------------
// Bus type aliases — referenced from other modules.
// ---------------------------------------------------------------------------
type RawI2c   = esp_hal::i2c::master::I2c<'static, esp_hal::Async>;
type I2cMutex = Mutex<NoopRawMutex, RawI2c>;
pub type I2cBus = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice<
    'static, NoopRawMutex, RawI2c,
>;

type RawSpi = esp_hal::spi::master::Spi<'static, esp_hal::Async>;
pub type SpiDev = ExclusiveDevice<RawSpi, Output<'static>, Delay>;

/// Boot-time fatal: log the offending subsystem + error, wait 5 s so the
/// log line actually flushes over UART, then trigger a software reset.
/// Diverges (`-> !`) so callers can use it as a fall-through expression.
///
/// Never returns. Spinning `loop {}` after the reset call is unreachable
/// in practice but lets the compiler keep the `!` return type without
/// relying on `software_reset`'s signature being `-> !` (it isn't on
/// esp-hal 0.23, which returns `()`).
async fn fatal_init<E: core::fmt::Debug + ?Sized>(subsystem: &str, err: &E) -> ! {
    log::error!("FATAL: {} init failed: {:?} — rebooting in 5 s", subsystem, err);
    embassy_time::Timer::after(embassy_time::Duration::from_secs(5)).await;
    esp_hal::reset::software_reset();
    loop { embassy_futures::yield_now().await; }
}

/// Boot-time I²C bus scan. Pings every 7-bit address in the valid range
/// (0x08..=0x77) by issuing a 0-byte read; an ACK means a device lives
/// there. Pure diagnostic — no side effects on bus state.
async fn i2c_scan<B>(bus: &mut B, label: &str) -> u8
where
    B: embedded_hal_async::i2c::I2c,
{
    info!("i2c scan [{}]: probing 0x08..0x77 ...", label);
    let mut found = 0u8;
    for addr in 0x08u8..=0x77 {
        let mut buf = [0u8; 1];
        if bus.read(addr, &mut buf).await.is_ok() {
            info!("i2c scan [{}]: device @ 0x{:02X}", label, addr);
            found += 1;
        }
    }
    info!("i2c scan [{}]: {} device(s) found", label, found);
    found
}



#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // -----------------------------------------------------------------
    // Bring up the chip + logging.
    // -----------------------------------------------------------------
    let peripherals = esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(CpuClock::max()),
    );
    esp_println::logger::init_logger_from_env();
    esp_alloc::heap_allocator!(72 * 1024);

    info!("KC868-A6 firmware starting");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    // Pluck the WDT out of TIMG0 before we surrender `timg0.timer0` to
    // esp-wifi — they're independent sub-peripherals of the same block
    // but Rust ownership won't let us touch `timg0.wdt` once any field
    // has been moved if TimerGroup is `Drop`. Do this first.
    let hw_wdt = timg0.wdt;

    // -----------------------------------------------------------------
    // I2C bus — shared between PCF8574 (relays), input PCF8574, OLED, RTC.
    // KC868-A6 v1.4SP pinout: GPIO 4 (SDA) / GPIO 15 (SCL).
    // -----------------------------------------------------------------
    let i2c = match esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default()
            .with_frequency(bsp::i2c::FREQ_HZ.Hz()),
    ) {
        Ok(i2c) => i2c
            .with_sda(peripherals.GPIO4)
            .with_scl(peripherals.GPIO15)
            .into_async(),
        Err(e) => fatal_init("i2c", &e).await,
    };

    static I2C_MUTEX: StaticCell<I2cMutex> = StaticCell::new();
    let i2c_mutex: &'static I2cMutex = I2C_MUTEX.init(Mutex::new(i2c));

    let relay_i2c   = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_mutex);
    let display_i2c = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_mutex);
    let input_i2c   = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_mutex);

    // -----------------------------------------------------------------
    // SPI bus + MAX31865 — disabled until the Pt100 hardware is wired.
    // When you populate the sensor, uncomment, then spawn `temperature_task`
    // below. The SPI pins are SCK=14, MISO=12, MOSI=13, CS=GPIO 2 (note:
    // GPIO 15 is I²C SCL on this board, cannot be reused for CS).
    // -----------------------------------------------------------------
    // let spi = esp_hal::spi::master::Spi::new(...) ...
    // let cs = Output::new(peripherals.GPIO2, Level::High);
    // let max31865 = max31865::Max31865::new(spi_dev);

    // -----------------------------------------------------------------
    // WiFi + embassy-net.
    // -----------------------------------------------------------------
    let mut rng = Rng::new(peripherals.RNG);
    let seed = ((rng.random() as u64) << 32) | rng.random() as u64;

    let wifi_init = match esp_wifi::init(timg0.timer0, rng, peripherals.RADIO_CLK) {
        Ok(w) => w,
        Err(e) => fatal_init("wifi", &e).await,
    };
    static WIFI_INIT: StaticCell<esp_wifi::EspWifiController<'static>> = StaticCell::new();
    let wifi_init = WIFI_INIT.init(wifi_init);

    let (wifi_device, wifi_controller) = match esp_wifi::wifi::new_with_mode(
        wifi_init,
        peripherals.WIFI,
        esp_wifi::wifi::WifiStaDevice,
    ) {
        Ok(pair) => pair,
        Err(e) => fatal_init("wifi mode", &e).await,
    };

    let stack = wifi::start(&spawner, wifi_controller, wifi_device, seed);

    // -----------------------------------------------------------------
    // Inter-task channels.
    // -----------------------------------------------------------------
    static RELAY_CH: StaticCell<relays::RelayChannel> = StaticCell::new();
    let relay_ch = RELAY_CH.init(relays::RelayChannel::new());
    let relay_tx = relay_ch.sender();
    let relay_rx = relay_ch.receiver();

    // -----------------------------------------------------------------
    // I2C scan on the main bus (post-init, on the configured pin pair).
    // -----------------------------------------------------------------
    {
        let mut scan_bus =
            embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_mutex);
        i2c_scan(&mut scan_bus, "main").await;
    }

    // -----------------------------------------------------------------
    // Spawn workers.
    // -----------------------------------------------------------------
    let relays = match relays::Relays::new(relay_i2c, bsp::i2c_addr::RELAY_EXPANDER).await {
        Ok(r) => r,
        // We cannot prove relay state without I²C — refuse to run.
        // PCF8574 powers up tri-stated → port reads 0xFF → active-low
        // chain interprets as all-OFF, which is the safe state.
        Err(e) => fatal_init("relay expander", &e).await,
    };
    spawner.must_spawn(relays::relay_task(relays, relay_rx));
    spawner.must_spawn(display::display_task(display_i2c, stack));
    spawner.must_spawn(inputs::input_task(
        input_i2c, bsp::i2c_addr::INPUT_EXPANDER, relay_tx,
    ));
    spawner.must_spawn(hw_watchdog::hw_watchdog_task(hw_wdt));

    // Front-panel GPIO0 button → relay running-lights self-test.
    // On-board R52 (10 kΩ) pulls it high; a press drives it low.
    let test_button = Input::new(peripherals.GPIO0, Pull::Up);
    spawner.must_spawn(button::button_task(test_button, relay_tx));
    // spawner.must_spawn(temperature::temperature_task(max31865)); // no sensor

    // Wait for IP, then bring MQTT + comms watchdog up.
    wifi::wait_for_link(&stack).await;
    spawner.must_spawn(mqtt::mqtt_publisher_task(stack));
    spawner.must_spawn(mqtt::mqtt_subscriber_task(stack, relay_tx));
    spawner.must_spawn(watchdog::watchdog_task(stack, relay_tx));

    info!("startup complete");
}
