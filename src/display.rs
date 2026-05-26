//! SSD1306 128x64 OLED on the shared I²C bus (address 0x3C, P3 header).
//!
//! Renders a multi-line status page: IP, relay ON-mask, safety state,
//! optional temperature. Self-heals: any init or flush error puts the
//! task back into init-retry mode rather than killing the task silently.

use core::fmt::Write as _;

use embassy_net::Stack;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use heapless::String;
use log::{info, warn};
use ssd1306::{
    mode::DisplayConfigAsync,
    rotation::DisplayRotation,
    size::DisplaySize128x64,
    I2CDisplayInterface, Ssd1306Async,
};

use crate::mqtt::TEMPERATURE;
use crate::safety;

const REFRESH:    Duration = Duration::from_millis(500);
const INIT_RETRY: Duration = Duration::from_secs(5);

#[embassy_executor::task]
pub async fn display_task(i2c: crate::I2cBus, stack: Stack<'static>) {
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    // Cache last good temperature so a transient sensor outage doesn't
    // blank the line. `None` until the first sample arrives.
    let mut last_temp_c: Option<f32> = None;

    'outer: loop {
        // ---- (re)initialise ------------------------------------------
        if let Err(e) = display.init().await {
            warn!("oled: init failed: {:?} — retry in {} s", e, INIT_RETRY.as_secs());
            Timer::after(INIT_RETRY).await;
            continue;
        }
        info!("oled: ready");

        // ---- refresh loop -------------------------------------------
        loop {
            if let Some(t) = TEMPERATURE.try_take() {
                last_temp_c = Some(t);
            }

            let _ = display.clear(BinaryColor::Off);

            // Line 1 — banner
            let _ = Text::new("KC868-A6", Point::new(0, 10), style).draw(&mut display);

            // Line 2 — IP address (or "no link" before DHCP)
            let mut buf: String<32> = String::new();
            if let Some(cfg) = stack.config_v4() {
                let _ = write!(buf, "IP: {}", cfg.address.address());
            } else {
                let _ = write!(buf, "IP: (no link)");
            }
            let _ = Text::new(buf.as_str(), Point::new(0, 24), style).draw(&mut display);

            // Line 3 — relay state
            let on_mask = crate::relays::current_on_mask();
            buf.clear();
            let _ = write!(buf, "R:{}{}{}{}{}{}",
                glyph(on_mask, 0), glyph(on_mask, 1), glyph(on_mask, 2),
                glyph(on_mask, 3), glyph(on_mask, 4), glyph(on_mask, 5));
            let _ = Text::new(buf.as_str(), Point::new(0, 38), style).draw(&mut display);

            // Line 4 — temperature
            buf.clear();
            match last_temp_c {
                Some(t) => { let _ = write!(buf, "T: {:.1} C", t); }
                None    => { let _ = write!(buf, "T: --.- C"); }
            }
            let _ = Text::new(buf.as_str(), Point::new(0, 52), style).draw(&mut display);

            // Line 5 — safety
            let _ = Text::new(
                if safety::is_locked() { "[!] SAFETY LOCK" } else { "OK" },
                Point::new(0, 64),
                style,
            ).draw(&mut display);

            if let Err(e) = display.flush().await {
                warn!("oled: flush failed: {:?} — reinitialising", e);
                continue 'outer;
            }

            Timer::after(REFRESH).await;
        }
    }
}

fn glyph(mask: u8, bit: u8) -> char {
    if mask & (1 << bit) != 0 { '#' } else { '.' }
}
