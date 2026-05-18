//! SSD1306 128x64 OLED on the shared I²C bus (address 0x3C, P3 header).
//!
//! Owns its own `I2cDevice` over the embassy shared-bus mutex, so it can
//! run concurrently with the relay task without ever serialising at the
//! application level.

use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use log::{info, warn};
use ssd1306::{
    mode::DisplayConfigAsync,
    prelude::*,
    rotation::DisplayRotation,
    size::DisplaySize128x64,
    I2CDisplayInterface, Ssd1306Async,
};

#[embassy_executor::task]
pub async fn display_task(i2c: crate::I2cBus) {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    if let Err(e) = display.init().await {
        warn!("oled: init failed: {:?}", e);
        return;
    }
    info!("oled: ready");

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let _ = display.clear(BinaryColor::Off);
    let _ = Text::new("Hello World", Point::new(24, 32), style).draw(&mut display);

    if let Err(e) = display.flush().await {
        warn!("oled: flush failed: {:?}", e);
    } else {
        info!("oled: drew \"Hello World\"");
    }

    // Park the task — re-draw periodically as a liveness indicator and to
    // shake off any glitched frame after a brief power dip on the panel.
    loop {
        Timer::after(Duration::from_secs(10)).await;
        let _ = display.clear(BinaryColor::Off);
        let _ = Text::new("Hello World", Point::new(24, 32), style).draw(&mut display);
        let _ = display.flush().await;
    }
}
