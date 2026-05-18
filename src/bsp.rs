//! Board support: pin map and I2C addresses for KC868-A6.
//!
//! The KC868-A6 schematic does not label every ESP32 GPIO next to the net
//! name, so the values below come from the standard KinCony reference. If a
//! peripheral misbehaves, verify the pin against your specific unit and
//! adjust here — every other module imports its pins from this file.

#![allow(dead_code)] // pin constants are documentation-as-code; not all are consumed yet

/// PCF8574 I²C addresses (base 0x20; "PCF8574AT" silkscreen here means
/// SOIC package, not the address-variant PCF8574A — confirmed by the
/// working KinCony Arduino reference which uses 0x24).
/// Strap pins per schematic: A0=GND, A1=GND, A2=3V3 → +4 over the base.
pub mod i2c_addr {
    /// Output expander driving the 6 relays through 74HCT14 + ULN2003A.
    pub const RELAY_EXPANDER: u8 = 0x24;
    /// Input expander reading the 6 opto-isolated 12 V digital inputs.
    pub const INPUT_EXPANDER: u8 = 0x22;
}

/// I2C bus — SDA / SCL routed to PCF8574 expanders, OLED slot, RTC.
pub mod i2c {
    // KC868-A6 v1.4SP pinout (verified against KinCony Arduino reference).
    // Non-standard: SCL on GPIO 15, not GPIO 5.
    pub const SDA: u8 = 4;
    pub const SCL: u8 = 15;
    pub const FREQ_HZ: u32 = 100_000;
}

/// SPI bus — wired to the LoRa SX127x footprint and the nRF24L01 header.
/// We re-use it for the MAX31865 Pt100 frontend.
pub mod spi {
    pub const SCK:  u8 = 14;
    pub const MISO: u8 = 12;
    pub const MOSI: u8 = 13;
    /// MAX31865 chip-select. Cannot use GPIO 15 — that's I²C SCL here.
    /// GPIO 2 is a strapping pin but free for general use after boot.
    pub const CS_MAX31865: u8 = 2;
    pub const FREQ_HZ: u32 = 1_000_000;
}

/// Relay bit positions inside the PCF8574 output port.
/// Schematic nets OT1..OT6 → P0..P5; OT7/OT8 are unused for relays.
///
/// **Active-low logic**: PCF8574 → 74HCT14 (inverter) → ULN2003A. The
/// single inversion means writing a `0` bit energises the relay, a `1`
/// bit de-energises it. Initial port value is `0xFF` so every relay
/// is OFF at power-up (critical — otherwise mains loads switch on for
/// the boot duration).
pub mod relay {
    pub const COUNT: usize = 6;
    pub const BITS: [u8; COUNT] = [0, 1, 2, 3, 4, 5];
    pub const IDLE_PORT_VALUE: u8 = 0xFF;
}

/// MAX31865 module configuration.
pub mod max31865 {
    /// Reference resistor on the breakout (Adafruit board = 430 Ω, generic
    /// Chinese boards usually 400 Ω). Used in the RTD → Ω conversion.
    pub const RREF_OHMS: f32 = 430.0;
    /// Nominal RTD resistance at 0 °C.
    pub const R0_OHMS:   f32 = 100.0;
    /// Sampling period for the temperature task.
    pub const SAMPLE_PERIOD_MS: u64 = 1000;
}

/// MQTT topic layout. Keep short — embedded clients are RAM-bound.
pub mod mqtt_topic {
    pub const BASE:        &str = "kc868";
    pub const TEMPERATURE: &str = "kc868/temperature";
    /// Single-level wildcard subscription for relay commands:
    ///   kc868/relay/<0..5>/set   payload: "0" | "1"
    pub const RELAY_CMD_SUB: &str = "kc868/relay/+/set";
    pub const RELAY_STATE_PREFIX: &str = "kc868/relay";
    pub const STATUS:       &str = "kc868/status";
}
