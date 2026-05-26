//! Hardware Task Watchdog (TIMG0 MWDT).
//!
//! Single-stage 8-second watchdog. Fed every 1 second by
//! `hw_watchdog_task`. If the executor itself wedges (an `await` on a
//! future that never resolves, kernel-level panic outside our handlers,
//! a critical task panicking in a way that disables interrupts), this
//! task no longer runs, the WDT is not fed, and the chip hard-resets
//! within `STAGE0_TIMEOUT`.
//!
//! Per-task liveness watchdogs are intentionally **not** added here:
//! `relay_task` and `mqtt_subscriber_task` legitimately block on a
//! channel/socket and would false-trip a naive per-task watch. The
//! comms-layer `watchdog::watchdog_task` already covers the most
//! likely runtime failure (network/MQTT loss).

use embassy_time::{Duration, Timer};
use esp_hal::peripherals::TIMG0;
use esp_hal::time::ExtU64;
use esp_hal::timer::timg::{MwdtStage, MwdtStageAction, Wdt};

const STAGE0_TIMEOUT_SEC: u64 = 8;
const FEED_PERIOD:        Duration = Duration::from_secs(1);

#[embassy_executor::task]
pub async fn hw_watchdog_task(mut wdt: Wdt<TIMG0>) {
    wdt.set_timeout(MwdtStage::Stage0, (STAGE0_TIMEOUT_SEC * 1_000_000).micros());
    wdt.set_stage_action(MwdtStage::Stage0, MwdtStageAction::ResetSystem);
    wdt.enable();

    loop {
        wdt.feed();
        Timer::after(FEED_PERIOD).await;
    }
}
