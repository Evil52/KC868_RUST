//! Front-panel button (GPIO0) → relay self-test.
//!
//! The KC868-A6 has two push-buttons next to the ESP32 module. One is
//! wired to the chip's `EN` line — that's a pure **hardware reset**, not
//! readable from firmware. The other is wired to **GPIO0** (the classic
//! ESP32 BOOT strap) and *is* readable once boot has finished.
//!
//! On each press of the GPIO0 button we run a "running-lights" relay
//! self-test: K1 → K2 → … → K6, one at a time, 200 ms apart, lighting
//! each relay's indicator LED in turn, then all-off. Useful for a quick
//! field check that every channel and its LED actually switch.
//!
//! GPIO0 quirks handled:
//!   * It's a strap pin — must be HIGH at boot or the chip enters
//!     download mode. The on-board 10 kΩ pull-up (R52) guarantees that;
//!     we only *read* it afterwards.
//!   * Buttons bounce — we debounce with a short settle delay and
//!     require the level to still be asserted after it.
//!   * If the safety interlock is engaged, the chase is refused (the
//!     relay task would reject the SetMask anyway, but we skip it
//!     cleanly and log a clear reason).

use embassy_time::{Duration, Timer};
use esp_hal::gpio::Input;
use log::{info, warn};

use crate::bsp::relay::COUNT;
use crate::relays::{RelayCommand, RelayTx};
use crate::safety;

const DEBOUNCE: Duration = Duration::from_millis(50);
const CHASE_STEP: Duration = Duration::from_millis(800);

#[embassy_executor::task]
pub async fn button_task(mut button: Input<'static>, relay_tx: RelayTx) {
    info!("button: GPIO0 self-test button ready");
    loop {
        // Idle level is HIGH (pull-up); a press pulls it LOW.
        button.wait_for_falling_edge().await;

        // Debounce: re-check after the contacts settle.
        Timer::after(DEBOUNCE).await;
        if button.is_high() {
            continue; // bounce / spurious edge
        }

        if safety::is_locked() {
            warn!("button: chase test refused — safety interlock engaged");
        } else {
            info!("button: running-lights self-test");
            chase(&relay_tx).await;
        }

        // Wait for release (+ settle) so a single press triggers once.
        button.wait_for_high().await;
        Timer::after(DEBOUNCE).await;
    }
}

/// K1 → K6 one at a time, then all-off. `SetMask(1<<i)` energises exactly
/// the i-th relay and de-energises the rest in a single I²C write, so the
/// pattern is glitch-free and bypasses the per-channel command debounce.
async fn chase(relay_tx: &RelayTx) {
    for i in 0..COUNT as u8 {
        relay_tx.send(RelayCommand::SetMask(1 << i)).await;
        Timer::after(CHASE_STEP).await;
    }
    relay_tx.send(RelayCommand::AllOff).await;
}
