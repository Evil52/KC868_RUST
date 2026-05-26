//! Home Assistant MQTT Discovery — publish the JSON config blobs that
//! make HA auto-create:
//!   * 6 `switch` entities (one per relay)
//!   * 1 `binary_sensor` for the safety interlock
//!   * 1 `sensor` for temperature (lives even when the sensor isn't
//!     wired — HA will just show "unavailable")
//!
//! The retained configs are published once per boot, after the MQTT
//! publisher session is established. HA discovers them on its next
//! `homeassistant/+/+/config` scan (or immediately if it's already
//! running). If HA wasn't online at our boot, the retained nature
//! means it picks them up later.
//!
//! Each entity references our common `availability_topic =
//! kc868/status`, which the publisher keeps as `"online"` while
//! connected. The LWT replaces it with `"offline"` on any abrupt
//! disconnect, so HA marks the entities unavailable automatically.

use core::fmt::Write as _;

use heapless::String;
use log::{info, warn};
use rust_mqtt::client::client::MqttClient;
use rust_mqtt::packet::v5::publish_packet::QualityOfService;
use rust_mqtt::utils::rng_generator::CountingRng;

use crate::bsp::{mqtt_topic, relay};

/// Publish all discovery configs in one shot. Best-effort: a single
/// failure is logged and skipped, the rest are still attempted.
pub async fn publish_all<'a, T>(client: &mut MqttClient<'a, T, 5, CountingRng>)
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    info!("ha-discovery: publishing configs");

    for idx in 0..relay::COUNT as u8 {
        publish_switch(client, idx).await;
    }
    publish_safety_binary_sensor(client).await;
    publish_temperature_sensor(client).await;

    info!("ha-discovery: done");
}

async fn publish_switch<'a, T>(
    client: &mut MqttClient<'a, T, 5, CountingRng>,
    idx: u8,
)
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    // Topic: homeassistant/switch/kc868_a6/relay_<N>/config
    let mut topic: String<96> = String::new();
    let _ = write!(
        topic,
        "{}/switch/{}/relay_{}/config",
        mqtt_topic::HA_DISCOVERY_PREFIX, mqtt_topic::DEVICE_ID, idx,
    );

    // Compact JSON — HA accepts both `device` short-form and full names.
    let mut payload: String<768> = String::new();
    let built = write!(payload,
        "{{\"name\":\"Relay {n}\",\
          \"unique_id\":\"{dev}_relay_{i}\",\
          \"command_topic\":\"{base}/relay/{i}/set\",\
          \"state_topic\":\"{base}/relay/{i}/state\",\
          \"payload_on\":\"ON\",\
          \"payload_off\":\"OFF\",\
          \"state_on\":\"ON\",\
          \"state_off\":\"OFF\",\
          \"availability_topic\":\"{base}/status\",\
          \"payload_available\":\"online\",\
          \"payload_not_available\":\"offline\",\
          \"device\":{device_block}\
        }}",
        n = idx + 1,
        i = idx,
        dev = mqtt_topic::DEVICE_ID,
        base = mqtt_topic::BASE,
        device_block = DEVICE_BLOCK,
    );

    if built.is_err() {
        warn!("ha-discovery: switch {} payload truncated — skipped", idx);
        return;
    }
    publish_retained(client, topic.as_str(), payload.as_bytes(), "switch").await;
}

async fn publish_safety_binary_sensor<'a, T>(client: &mut MqttClient<'a, T, 5, CountingRng>)
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let mut topic: String<96> = String::new();
    let _ = write!(
        topic,
        "{}/binary_sensor/{}/safety/config",
        mqtt_topic::HA_DISCOVERY_PREFIX, mqtt_topic::DEVICE_ID,
    );

    let mut payload: String<768> = String::new();
    let built = write!(payload,
        "{{\"name\":\"Safety Interlock\",\
          \"unique_id\":\"{dev}_safety\",\
          \"state_topic\":\"{base}/safety/state\",\
          \"payload_on\":\"locked\",\
          \"payload_off\":\"ok\",\
          \"device_class\":\"safety\",\
          \"availability_topic\":\"{base}/status\",\
          \"payload_available\":\"online\",\
          \"payload_not_available\":\"offline\",\
          \"device\":{device_block}\
        }}",
        dev = mqtt_topic::DEVICE_ID,
        base = mqtt_topic::BASE,
        device_block = DEVICE_BLOCK,
    );

    if built.is_err() {
        warn!("ha-discovery: safety payload truncated — skipped");
        return;
    }
    publish_retained(client, topic.as_str(), payload.as_bytes(), "safety").await;
}

async fn publish_temperature_sensor<'a, T>(client: &mut MqttClient<'a, T, 5, CountingRng>)
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let mut topic: String<96> = String::new();
    let _ = write!(
        topic,
        "{}/sensor/{}/temperature/config",
        mqtt_topic::HA_DISCOVERY_PREFIX, mqtt_topic::DEVICE_ID,
    );

    let mut payload: String<768> = String::new();
    let built = write!(payload,
        "{{\"name\":\"Temperature\",\
          \"unique_id\":\"{dev}_temp\",\
          \"state_topic\":\"{temp_topic}\",\
          \"unit_of_measurement\":\"\u{00B0}C\",\
          \"device_class\":\"temperature\",\
          \"state_class\":\"measurement\",\
          \"suggested_display_precision\":1,\
          \"availability_topic\":\"{base}/status\",\
          \"payload_available\":\"online\",\
          \"payload_not_available\":\"offline\",\
          \"device\":{device_block}\
        }}",
        dev = mqtt_topic::DEVICE_ID,
        base = mqtt_topic::BASE,
        temp_topic = mqtt_topic::TEMPERATURE,
        device_block = DEVICE_BLOCK,
    );

    if built.is_err() {
        warn!("ha-discovery: temperature payload truncated — skipped");
        return;
    }
    publish_retained(client, topic.as_str(), payload.as_bytes(), "temperature").await;
}

async fn publish_retained<'a, T>(
    client: &mut MqttClient<'a, T, 5, CountingRng>,
    topic: &str, payload: &[u8], label: &str,
)
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    if let Err(e) = client
        .send_message(topic, payload, QualityOfService::QoS0, true)
        .await
    {
        warn!("ha-discovery: '{}' failed: {:?}", label, e);
    }
}

/// Shared `"device"` JSON block, embedded inside every config payload.
/// One block per physical KC868-A6 → HA groups all entities under a
/// single device card.
const DEVICE_BLOCK: &str = "{\
    \"identifiers\":[\"kc868_a6\"],\
    \"manufacturer\":\"KinCony\",\
    \"model\":\"KC868-A6\",\
    \"name\":\"KC868-A6 Controller\",\
    \"sw_version\":\"0.1.0\"\
}";
