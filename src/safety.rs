//! Global safety interlock.
//!
//! When engaged, every `RelayCommand` other than `AllOff` is rejected by
//! the relay task. The interlock is set by:
//!   * `input_task` — opto-input opened (E-Stop / door / light curtain)
//!   * `safety_watchdog_task` — MQTT or WiFi link lost beyond grace period
//!
//! And cleared by either a successful `safety/reset` MQTT command **and**
//! all opto-inputs being closed again, or by a hardware reset.
//!
//! A bare `AtomicBool` is enough — there is no protocol state, just a
//! single boolean gate. We expose helper functions instead of the atomic
//! directly so the call-sites read naturally.

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

static LOCKED: AtomicBool = AtomicBool::new(false);

/// Signalled every time the interlock state flips. Other tasks (display,
/// publisher) can `wait()` on it to refresh their view.
pub static CHANGED: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[inline]
pub fn is_locked() -> bool {
    LOCKED.load(Ordering::Acquire)
}

/// Engage the interlock. Idempotent. Returns `true` if this call actually
/// flipped the state (so the caller can decide whether to log/publish).
pub fn lock(reason: &'static str) -> bool {
    let flipped = !LOCKED.swap(true, Ordering::AcqRel);
    if flipped {
        log::warn!("safety: LOCKED — {}", reason);
        CHANGED.signal(true);
        // Wake the MQTT publisher so the new safety state propagates
        // immediately instead of waiting for the next heartbeat tick.
        crate::relays::STATE_CHANGED.signal(());
    }
    flipped
}

/// Release the interlock. Same semantics as `lock`.
pub fn unlock(reason: &'static str) -> bool {
    let flipped = LOCKED.swap(false, Ordering::AcqRel);
    if flipped {
        log::info!("safety: unlocked — {}", reason);
        CHANGED.signal(false);
        crate::relays::STATE_CHANGED.signal(());
    }
    flipped
}
