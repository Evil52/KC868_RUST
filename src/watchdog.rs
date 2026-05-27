//! Communication-layer fail-safe watchdog.
//!
//! Watches the WiFi link and the MQTT publisher session: if either has
//! been down longer than its grace period, engages `safety::lock` and
//! emits `RelayCommand::AllOff`. Once engaged, the lock is **not**
//! cleared automatically — even when comms come back, the operator must
//! issue `kc868/safety/reset` (handled in `inputs::try_safety_reset`).
//!
//! The rationale is matching the field-safety expectation: a remote
//! supervisory client should not be able to silently miss a relay state
//! during the outage and then assume the controller is doing what it
//! thinks. Forcing AllOff + requiring an explicit reset closes that
//! window.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_net::Stack;
use embassy_time::{Duration, Instant, Timer};
use log::warn;

use crate::relays::{RelayCommand, RelayTx};
use crate::safety;

const POLL: Duration = Duration::from_secs(1);

/// Grace period after the MQTT publisher last successfully sent
/// something. Heartbeat publishes every 5 s, so 30 s allows ~6 missed
/// heartbeats before we trip.
pub const MQTT_GRACE: Duration = Duration::from_secs(30);

/// Grace period after the WiFi/IP link last looked healthy.
pub const WIFI_GRACE: Duration = Duration::from_secs(60);

/// Timestamps as `u32` millis since boot. ESP32 (Xtensa LX6) has no
/// hardware `AtomicU64`. u32 wraps at ~49.7 days — well past the
/// heartbeat period, so wraparound does not affect the freshness check.
/// `u32::MAX` is the "never" sentinel: suppresses the watchdog until
/// the first successful publish so a fresh boot does not trip before
/// MQTT even comes up.
static MQTT_LAST_OK_MS: AtomicU32 = AtomicU32::new(u32::MAX);
static WIFI_LAST_OK_MS: AtomicU32 = AtomicU32::new(u32::MAX);

#[inline]
fn now_ms_u32() -> u32 {
    // Saturating cast — u64 → u32. The watchdog cares about
    // differences, not absolute values, and the 49-day wrap is
    // harmless for that.
    Instant::now().as_millis() as u32
}

/// Called by the MQTT publisher on every successful publish.
#[inline]
pub fn pet_mqtt() {
    MQTT_LAST_OK_MS.store(now_ms_u32(), Ordering::Release);
}

#[embassy_executor::task]
pub async fn watchdog_task(stack: Stack<'static>, relay_tx: RelayTx) {
    loop {
        let now = now_ms_u32();

        if stack.config_v4().is_some() {
            WIFI_LAST_OK_MS.store(now, Ordering::Release);
        }

        let mqtt_age = age_since(MQTT_LAST_OK_MS.load(Ordering::Acquire), now);
        let wifi_age = age_since(WIFI_LAST_OK_MS.load(Ordering::Acquire), now);

        let mqtt_down = mqtt_age.map_or(false, |d| d > MQTT_GRACE);
        let wifi_down = wifi_age.map_or(false, |d| d > WIFI_GRACE);

        // Pick the reason via an `if` chain — keeps the unreachable
        // `(false, false)` arm out of the source so there's no panic
        // path even at -O0, and produces clearer code for the reader.
        let reason: Option<&'static str> = if mqtt_down && wifi_down {
            Some("WiFi + MQTT lost")
        } else if mqtt_down {
            Some("MQTT publisher lost")
        } else if wifi_down {
            Some("WiFi link lost")
        } else {
            None
        };

        if let Some(reason) = reason {
            if safety::lock(reason) {
                warn!(
                    "watchdog: locking — {} (mqtt_age={:?}, wifi_age={:?})",
                    reason, mqtt_age, wifi_age,
                );
                relay_tx.send(RelayCommand::AllOff).await;
            }
        }

        Timer::after(POLL).await;
    }
}

fn age_since(stamp_ms: u32, now_ms: u32) -> Option<Duration> {
    if stamp_ms == u32::MAX {
        // Sentinel: haven't seen the subsystem alive yet — don't trip.
        None
    } else {
        Some(Duration::from_millis(now_ms.wrapping_sub(stamp_ms) as u64))
    }
}
