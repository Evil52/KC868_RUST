#![no_std]
#![no_main]
//! Host-style unit tests compiled for the ESP32 target (no_std).
//!
//! These tests exercise pure logic without hardware:
//!   * Pt100 resistance → temperature conversion
//!   * PCF8574 shadow register bit manipulation
//!   * Relay active-low bit maths
//!   * MQTT topic parsing
//!   * MAX31865 raw data conversion
//!
//! Build and flash as a regular binary:
//!   cargo build --release --bin unit_tests
//!   espflash flash --monitor target/xtensa-esp32-none-elf/release/unit_tests
//!
//! Each test module has a `run_all() -> Result<(), &'static str>` entry
//! point.  A passing test returns Ok(()); a failing test returns Err(msg).
//!
//! NOTE: #[panic_handler] is provided by esp-backtrace (dependency in
//! Cargo.toml), so we do NOT define our own here.

use esp_println::println;

macro_rules! check {
    ($cond:expr, $msg:expr) => {
        if !($cond) {
            return Err($msg);
        }
    };
    ($cond:expr) => {
        if !($cond) {
            return Err("assertion failed");
        }
    };
}

macro_rules! check_eq {
    ($left:expr, $right:expr $(, $msg:expr)?) => {{
        let l = $left;
        let r = $right;
        if l != r {
            let msg = core::concat!("expected ", core::stringify!($left), " == ", core::stringify!($right));
            #[allow(unused_variables)]
            let msg: &str = msg;
            $(let msg: &str = $msg;)?
            return Err(msg);
        }
    }};
}

// ============================================================================
// Pt100: Callendar–Van Dusen conversion
// ============================================================================

mod pt100_tests {
    use libm::sqrtf;

    const A: f32 = 3.9083e-3;
    const B: f32 = -5.775e-7;
    const C: f32 = -4.183e-12;
    const R0: f32 = 100.0;

    fn resistance_to_celsius(r_ohms: f32, r0: f32) -> f32 {
        let ratio = r_ohms / r0;
        let disc = A * A - 4.0 * B * (1.0 - ratio);
        let t_pos = (-A + sqrtf(disc)) / (2.0 * B);

        if t_pos >= 0.0 {
            return t_pos;
        }

        let mut t = t_pos;
        for _ in 0..6 {
            let t2 = t * t;
            let t3 = t2 * t;
            let f = r0 * (1.0 + A * t + B * t2 + C * (t - 100.0) * t3) - r_ohms;
            let df = r0 * (A + 2.0 * B * t + C * (4.0 * t3 - 300.0 * t2));
            if df.abs() < f32::EPSILON {
                break;
            }
            let next = t - f / df;
            if (next - t).abs() < 1e-4 {
                return next;
            }
            t = next;
        }
        t
    }

    pub fn run_all() -> Result<(), &'static str> {
        // 0 °C → R0
        let t = resistance_to_celsius(R0, R0);
        check!((t - 0.0).abs() < 0.1, "0 °C mismatch");

        // 100 °C → ≈138.51 Ω
        let t = resistance_to_celsius(138.51, R0);
        check!((t - 100.0).abs() < 0.5, "100 °C mismatch");

        // 200 °C → ≈175.86 Ω
        let t = resistance_to_celsius(175.86, R0);
        check!((t - 200.0).abs() < 0.5, "200 °C mismatch");

        // 500 °C → ≈280.98 Ω
        let t = resistance_to_celsius(280.98, R0);
        check!((t - 500.0).abs() < 1.0, "500 °C mismatch");

        // -50 °C → ≈80.31 Ω
        let t = resistance_to_celsius(80.31, R0);
        check!((t - (-50.0)).abs() < 0.5, "-50 °C mismatch");

        // -100 °C → ≈60.26 Ω
        let t = resistance_to_celsius(60.26, R0);
        check!((t - (-100.0)).abs() < 0.5, "-100 °C mismatch");

        // -200 °C → ≈18.52 Ω
        let t = resistance_to_celsius(18.52, R0);
        check!((t - (-200.0)).abs() < 1.0, "-200 °C mismatch");

        // Short circuit detection
        let t = resistance_to_celsius(0.0, R0);
        check!(t < -200.0, "short circuit should give T < -200");

        // Open circuit detection
        let t = resistance_to_celsius(400.0, R0);
        check!(t > 800.0, "open circuit should give T > 800");

        // Monotonicity
        let resistors = [60.0f32, 80.0, 100.0, 120.0, 150.0, 200.0];
        let mut prev = f32::NEG_INFINITY;
        for &r in &resistors {
            let t = resistance_to_celsius(r, R0);
            check!(t > prev, "non-monotonic");
            prev = t;
        }

        Ok(())
    }
}

// ============================================================================
// PCF8574 shadow register bit manipulation
// ============================================================================

mod pcf8574_tests {
    fn set_bit(shadow: u8, bit: u8, high: bool) -> u8 {
        let mask = 1u8 << bit;
        if high {
            shadow | mask
        } else {
            shadow & !mask
        }
    }

    pub fn run_all() -> Result<(), &'static str> {
        // set_bit high
        check_eq!(set_bit(0x00, 0, true), 0x01);
        check_eq!(set_bit(0x00, 7, true), 0x80);
        check_eq!(set_bit(0xF0, 3, true), 0xF8);

        // set_bit low
        check_eq!(set_bit(0xFF, 0, false), 0xFE);
        check_eq!(set_bit(0xFF, 7, false), 0x7F);
        check_eq!(set_bit(0x0F, 3, false), 0x07);

        // idempotent
        check_eq!(set_bit(0x01, 0, true), 0x01);
        check_eq!(set_bit(0xFE, 0, false), 0xFE);

        Ok(())
    }
}

// ============================================================================
// Relay active-low bit manipulation
// ============================================================================

mod relay_tests {
    const COUNT: usize = 6;
    const BITS: [u8; COUNT] = [0, 1, 2, 3, 4, 5];
    const IDLE_PORT_VALUE: u8 = 0xFF;

    fn apply_set(shadow: u8, index: u8, on: bool) -> Option<(u8, u8)> {
        if (index as usize) >= COUNT {
            return None;
        }
        let bit = BITS[index as usize];
        let high = !on;
        let mask = 1u8 << bit;
        let new = if high { shadow | mask } else { shadow & !mask };
        let on_bit = if !high { 1u8 << bit } else { 0 };
        Some((new, on_bit))
    }

    fn apply_set_mask(mask: u8) -> u8 {
        let active_low = !mask & 0x3F;
        active_low | !0x3F
    }

    pub fn run_all() -> Result<(), &'static str> {
        // IDLE state
        check_eq!(IDLE_PORT_VALUE, 0xFF);

        // Relay 0 ON
        let (port, on_bit) = apply_set(IDLE_PORT_VALUE, 0, true).ok_or("invalid index")?;
        check_eq!(port, 0xFE);
        check_eq!(on_bit, 0x01);

        // Relay 0 OFF
        let (port, on_bit) = apply_set(0xFE, 0, false).ok_or("invalid index")?;
        check_eq!(port, 0xFF);
        check_eq!(on_bit, 0x00);

        // Relay 5 ON
        let (port, on_bit) = apply_set(IDLE_PORT_VALUE, 5, true).ok_or("invalid index")?;
        check_eq!(port, 0xDF);
        check_eq!(on_bit, 0x20);

        // Invalid index rejected
        check!(apply_set(IDLE_PORT_VALUE, 6, true).is_none());
        check!(apply_set(IDLE_PORT_VALUE, 255, false).is_none());

        // Multiple relays
        let mut port = IDLE_PORT_VALUE;
        let (p, _) = apply_set(port, 0, true).ok_or("invalid index")?;
        port = p;
        let (p, _) = apply_set(port, 3, true).ok_or("invalid index")?;
        port = p;
        check_eq!(port, 0xF6);

        // SetMask all on (lower 4)
        check_eq!(apply_set_mask(0x0F), 0xF0);

        // SetMask all off
        check_eq!(apply_set_mask(0x00), 0xFF);

        // SetMask upper bits ignored
        check_eq!(apply_set_mask(0xFF), apply_set_mask(0x3F));
        check_eq!(apply_set_mask(0xFF), 0xC0);

        // on_bits calculation: (!shadow) & 0x3F
        let on_bits = (!0xF6u8) & 0x3F;
        check_eq!(on_bits, 0x09);

        Ok(())
    }
}

// ============================================================================
// MQTT topic parsing
// ============================================================================

mod mqtt_parsing_tests {
    fn parse_relay_cmd(topic: &str, payload: &[u8]) -> Option<(u8, bool)> {
        let suffix = topic.strip_prefix("kc868/relay")?;
        let suffix = suffix.trim_start_matches('/');
        let mut parts = suffix.split('/');
        let idx_str = parts.next()?;
        let cmd = parts.next()?;
        if cmd != "set" {
            return None;
        }
        let idx: u8 = idx_str.parse().ok()?;

        let on = match payload {
            b"1" | b"on" | b"ON" | b"true" => true,
            _ => false,
        };
        Some((idx, on))
    }

    pub fn run_all() -> Result<(), &'static str> {
        // Valid ON
        let (idx, on) = parse_relay_cmd("kc868/relay/3/set", b"1").ok_or("parse failed")?;
        check_eq!(idx, 3);
        check!(on);

        // Valid OFF
        let (idx, on) = parse_relay_cmd("kc868/relay/0/set", b"0").ok_or("parse failed")?;
        check_eq!(idx, 0);
        check!(!on);

        // Text ON payloads
        for payload in &[b"on" as &[u8], b"ON", b"true"] {
            let (_, on) = parse_relay_cmd("kc868/relay/1/set", payload).ok_or("parse failed")?;
            check!(on);
        }

        // Unknown payload → OFF
        let (_, on) = parse_relay_cmd("kc868/relay/2/set", b"garbage").ok_or("parse failed")?;
        check!(!on);

        // Wrong prefix
        check!(parse_relay_cmd("kc868/status", b"1").is_none());
        check!(parse_relay_cmd("kc868/temperature", b"25").is_none());
        check!(parse_relay_cmd("other/relay/0/set", b"1").is_none());

        // Missing "set"
        check!(parse_relay_cmd("kc868/relay/0", b"1").is_none());
        check!(parse_relay_cmd("kc868/relay/0/state", b"1").is_none());

        // Non-numeric index
        check!(parse_relay_cmd("kc868/relay/abc/set", b"1").is_none());
        check!(parse_relay_cmd("kc868/relay//set", b"1").is_none());

        // Empty payload
        let (_, on) = parse_relay_cmd("kc868/relay/0/set", b"").ok_or("parse failed")?;
        check!(!on);

        // Extra topic levels rejected
        check!(parse_relay_cmd("kc868/relay/0/set/extra", b"1").is_none());

        // Boundary indices
        let (idx, _) = parse_relay_cmd("kc868/relay/0/set", b"1").ok_or("parse failed")?;
        check_eq!(idx, 0);
        let (idx, _) = parse_relay_cmd("kc868/relay/5/set", b"1").ok_or("parse failed")?;
        check_eq!(idx, 5);

        // Out-of-range parsed but caller rejects
        let (idx, _) = parse_relay_cmd("kc868/relay/99/set", b"1").ok_or("parse failed")?;
        check_eq!(idx, 99);

        Ok(())
    }
}

// ============================================================================
// MAX31865 raw data conversion
// ============================================================================

mod max31865_tests {
    const RREF_OHMS: f32 = 430.0;

    fn code_to_resistance(code: u16, rref: f32) -> f32 {
        (code as f32) * rref / 32_768.0
    }

    pub fn run_all() -> Result<(), &'static str> {
        // Zero code
        let r = code_to_resistance(0, RREF_OHMS);
        check!((r - 0.0).abs() < 0.01);

        // Full scale (15-bit max)
        let r = code_to_resistance(0x7FFF, RREF_OHMS);
        check!((r - RREF_OHMS).abs() < 0.02);

        // Pt100 @ 0 °C expected code
        let expected_code = (100.0f32 * 32768.0 / RREF_OHMS) as u16;
        let r = code_to_resistance(expected_code, RREF_OHMS);
        check!((r - 100.0).abs() < 0.1);

        // Monotonic
        let mut prev = -1.0f32;
        let mut code = 0u16;
        while code <= 1000 {
            let r = code_to_resistance(code, RREF_OHMS);
            check!(r > prev);
            prev = r;
            code += 50;
        }

        Ok(())
    }
}

// ============================================================================
// Test runner entry point
// ============================================================================

#[no_mangle]
fn main() -> ! {
    println!("=== KC868-A6 Unit Tests ===\n");

    let mut passed: u32 = 0;
    let mut failed: u32 = 0;

    // Run each test module
    let suites: &[(&str, fn() -> Result<(), &'static str>)] = &[
        ("pt100 (Callendar-Van Dusen)", pt100_tests::run_all),
        ("PCF8574 (shadow register)", pcf8574_tests::run_all),
        ("Relay (active-low logic)", relay_tests::run_all),
        ("MQTT (topic parsing)", mqtt_parsing_tests::run_all),
        ("MAX31865 (raw conversion)", max31865_tests::run_all),
    ];

    for (name, run) in suites {
        match run() {
            Ok(()) => {
                println!("  {} ... PASS", name);
                passed += 1;
            }
            Err(msg) => {
                println!("  {} ... FAIL ({})", name, msg);
                failed += 1;
            }
        }
    }

    println!("\n---");
    println!("  {} passed, {} failed", passed, failed);

    if failed > 0 {
        println!("\n*** SOME TESTS FAILED ***");
    } else {
        println!("\n*** All tests passed ***");
    }

    loop {}
}
