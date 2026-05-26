//! MQTT layer — two independent clients on two TCP sockets:
//!
//!   * `mqtt_publisher_task`  — sends `kc868/status`, `kc868/temperature`.
//!   * `mqtt_subscriber_task` — listens on `kc868/relay/+/set`, forwards to
//!                              the relay channel.
//!
//! Why split? `rust_mqtt::MqttClient::receive_message` is not cancel-safe:
//! dropping the future mid-packet leaves the parser desynchronised and we
//! silently miss every following inbound PUBLISH. The cleanest fix is one
//! socket per direction — no `select` cancellation needed on receive, and
//! the publisher is free to block on its own Signal.
//!
//! Two MQTT sessions need two distinct client-ids; we suffix `-pub` / `-sub`
//! to the configured one.

use core::fmt::Write as _;
use core::str::FromStr;

use embassy_futures::select::{select3, Either3};
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_net::IpEndpoint;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use heapless::String;
use log::{info, warn};
use rust_mqtt::client::client::MqttClient;
use rust_mqtt::client::client_config::{ClientConfig, MqttVersion};
use rust_mqtt::packet::v5::publish_packet::QualityOfService;
use rust_mqtt::utils::rng_generator::CountingRng;

use crate::bsp::{mqtt_topic, relay};
use crate::ha_discovery;
use crate::relays::{self, RelayCommand, RelayTx};
use crate::safety;
use crate::watchdog;
use crate::wifi::NetStack;

const HOST:   &str = env!("MQTT_HOST");
const PORT:   &str = env!("MQTT_PORT");
const CLIENT: &str = env!("MQTT_CLIENT_ID");

const RECONNECT_DELAY: Duration = Duration::from_secs(2);
const TCP_TIMEOUT:     Duration = Duration::from_secs(20);
const TCP_KEEPALIVE:   Duration = Duration::from_secs(15);

/// Periodic application-level heartbeat publish. Keeps the watchdog
/// "petted" even when no temperature samples arrive — silence on the
/// MQTT publisher socket would otherwise trip the fail-safe.
const HEARTBEAT_PERIOD: Duration = Duration::from_secs(5);

/// Latest temperature sample — pt100 task signals, publisher reads.
pub static TEMPERATURE: Signal<CriticalSectionRawMutex, f32> = Signal::new();

// ---------------------------------------------------------------------------
// Publisher
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn mqtt_publisher_task(stack: NetStack) {
    let port: u16 = PORT.parse().unwrap_or(1883);
    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut mqtt_rx = [0u8; 1024];
    let mut mqtt_tx = [0u8; 1024];

    loop {
        let addr = match resolve(&stack, HOST).await {
            Some(ip) => ip,
            None => { Timer::after(RECONNECT_DELAY).await; continue; }
        };

        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(TCP_TIMEOUT));
        socket.set_keep_alive(Some(TCP_KEEPALIVE));

        info!("mqtt[pub]: connecting to {}:{}", addr, port);
        if let Err(e) = socket.connect(IpEndpoint::new(addr, port)).await {
            warn!("mqtt[pub]: tcp connect failed: {:?}", e);
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }

        let mut cfg: ClientConfig<'_, 5, CountingRng> =
            ClientConfig::new(MqttVersion::MQTTv5, CountingRng(20_000));
        cfg.add_max_subscribe_qos(QualityOfService::QoS0);
        cfg.add_client_id(concat!(env!("MQTT_CLIENT_ID"), "-pub"));
        cfg.max_packet_size = 1024;
        cfg.keep_alive = 30;
        // LWT — broker publishes "offline" (retained) on ungraceful
        // disconnect (TCP RST, keepalive timeout). The matching "online"
        // publish below overrides the retained state cleanly.
        cfg.add_will(mqtt_topic::STATUS, b"offline", true);

        let mut client = MqttClient::<_, 5, _>::new(
            socket, &mut mqtt_tx, 1024, &mut mqtt_rx, 1024, cfg,
        );

        if let Err(e) = client.connect_to_broker().await {
            warn!("mqtt[pub]: connect failed: {:?}", e);
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }
        info!("mqtt[pub]: connected");

        if client
            .send_message(mqtt_topic::STATUS, b"online", QualityOfService::QoS1, true)
            .await
            .is_ok()
        {
            watchdog::pet_mqtt();
        }

        // Discovery (HA auto-config) once per session; also re-emit the
        // current relay + safety state right away so HA has fresh data
        // before the user clicks anything.
        ha_discovery::publish_all(&mut client).await;
        if publish_all_relay_states(&mut client).await.is_err()
            || publish_safety_state(&mut client).await.is_err()
        {
            warn!("mqtt[pub]: initial state publish failed; will retry on next change");
        }

        // Inner loop — publish on whichever event fires first:
        //   * TEMPERATURE — fresh sample from temperature_task
        //   * STATE_CHANGED — relay state changed, re-emit per-relay
        //     retained state + safety state
        //   * heartbeat — application-level keep-alive for the watchdog
        let res = loop {
            let next = select3(
                TEMPERATURE.wait(),
                relays::STATE_CHANGED.wait(),
                Timer::after(HEARTBEAT_PERIOD),
            ).await;

            let send_result = match next {
                Either3::First(t) => publish_temperature(&mut client, t).await,
                Either3::Second(()) => {
                    let r = publish_all_relay_states(&mut client).await;
                    if r.is_ok() { publish_safety_state(&mut client).await } else { r }
                }
                Either3::Third(()) => {
                    // Heartbeat — retained "online" doubles as the
                    // freshness signal for the watchdog and any late
                    // supervisor that joins.
                    client.send_message(
                        mqtt_topic::STATUS,
                        b"online",
                        QualityOfService::QoS0,
                        true,
                    ).await
                }
            };

            match send_result {
                Ok(()) => watchdog::pet_mqtt(),
                Err(e) => break e,
            }
        };

        warn!("mqtt[pub]: session ended: {:?}", res);
        Timer::after(RECONNECT_DELAY).await;
    }
}

// ---------------------------------------------------------------------------
// Publish helpers
// ---------------------------------------------------------------------------

async fn publish_temperature<'a, T>(
    client: &mut MqttClient<'a, T, 5, CountingRng>,
    t: f32,
) -> Result<(), rust_mqtt::packet::v5::reason_codes::ReasonCode>
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let mut payload: String<16> = String::new();
    let _ = write_f32(&mut payload, t);
    client
        .send_message(
            mqtt_topic::TEMPERATURE,
            payload.as_bytes(),
            QualityOfService::QoS0,
            false,
        )
        .await
}

async fn publish_all_relay_states<'a, T>(
    client: &mut MqttClient<'a, T, 5, CountingRng>,
) -> Result<(), rust_mqtt::packet::v5::reason_codes::ReasonCode>
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let mask = relays::current_on_mask();
    for idx in 0..relay::COUNT as u8 {
        let on = mask & (1 << idx) != 0;
        let mut topic: String<48> = String::new();
        let _ = write!(topic, "{}/{}/state", mqtt_topic::RELAY_STATE_PREFIX, idx);
        let payload: &[u8] = if on { b"ON" } else { b"OFF" };
        client
            .send_message(topic.as_str(), payload, QualityOfService::QoS0, true)
            .await?;
    }
    Ok(())
}

async fn publish_safety_state<'a, T>(
    client: &mut MqttClient<'a, T, 5, CountingRng>,
) -> Result<(), rust_mqtt::packet::v5::reason_codes::ReasonCode>
where
    T: embedded_io_async::Read + embedded_io_async::Write,
{
    let payload: &[u8] = if safety::is_locked() { b"locked" } else { b"ok" };
    client
        .send_message(mqtt_topic::SAFETY_STATE, payload, QualityOfService::QoS0, true)
        .await
}

// ---------------------------------------------------------------------------
// Subscriber
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn mqtt_subscriber_task(stack: NetStack, relay_tx: RelayTx) {
    let port: u16 = PORT.parse().unwrap_or(1883);
    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut mqtt_rx = [0u8; 1024];
    let mut mqtt_tx = [0u8; 1024];

    loop {
        let addr = match resolve(&stack, HOST).await {
            Some(ip) => ip,
            None => { Timer::after(RECONNECT_DELAY).await; continue; }
        };

        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(TCP_TIMEOUT));
        socket.set_keep_alive(Some(TCP_KEEPALIVE));

        info!("mqtt[sub]: connecting to {}:{}", addr, port);
        if let Err(e) = socket.connect(IpEndpoint::new(addr, port)).await {
            warn!("mqtt[sub]: tcp connect failed: {:?}", e);
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }

        let mut cfg: ClientConfig<'_, 5, CountingRng> =
            ClientConfig::new(MqttVersion::MQTTv5, CountingRng(40_000));
        cfg.add_max_subscribe_qos(QualityOfService::QoS1);
        cfg.add_client_id(concat!(env!("MQTT_CLIENT_ID"), "-sub"));
        cfg.max_packet_size = 1024;
        // MQTT keep-alive **disabled** for the subscriber. `rust-mqtt 0.3`
        // doesn't auto-handle PINGRESP inside `receive_message()` — any
        // non-PUBLISH packet returns ImplementationSpecificError, which
        // we'd report as a session loss and reconnect every 30 s. Setting
        // `keep_alive = 0` tells the broker not to expect periodic PINGs
        // from us (MQTT 5 spec). Liveness is still protected:
        //   * TCP-keepalive (15 s, set on the socket below) catches dead
        //     links at the kernel level
        //   * The comms `watchdog_task` covers the application layer via
        //     the publisher heartbeat (every 5 s).
        cfg.keep_alive = 0;
        // Same LWT contract as the publisher — either client dying tells
        // the broker we're gone. Retained, so late subscribers see it.
        cfg.add_will(mqtt_topic::STATUS, b"offline", true);

        let mut client = MqttClient::<_, 5, _>::new(
            socket, &mut mqtt_tx, 1024, &mut mqtt_rx, 1024, cfg,
        );

        if let Err(e) = client.connect_to_broker().await {
            warn!("mqtt[sub]: connect failed: {:?}", e);
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }

        let subs: &[&str] = &[mqtt_topic::RELAY_CMD_SUB, mqtt_topic::SAFETY_RESET];
        let mut sub_failed = false;
        for topic in subs {
            if let Err(e) = client.subscribe_to_topic(topic).await {
                warn!("mqtt[sub]: subscribe to '{}' failed: {:?}", topic, e);
                sub_failed = true;
                break;
            }
            info!("mqtt[sub]: subscribed to {}", topic);
        }
        if sub_failed {
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }

        // Receive loop — never cancelled, so the parser stays in sync.
        let res = loop {
            match client.receive_message().await {
                Ok((topic, payload)) => handle_inbound(topic, payload, &relay_tx).await,
                Err(e) => break e,
            }
        };

        warn!("mqtt[sub]: session ended: {:?}", res);
        Timer::after(RECONNECT_DELAY).await;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

async fn handle_inbound(topic: &str, payload: &[u8], relay_tx: &RelayTx) {
    // Dispatch — two flat topics, parsed independently.
    if topic == mqtt_topic::SAFETY_RESET {
        let ok = crate::inputs::try_safety_reset();
        info!("mqtt[sub]: safety reset {}", if ok { "accepted" } else { "rejected" });
        return;
    }
    if let Some(suffix) = topic.strip_prefix(mqtt_topic::RELAY_STATE_PREFIX) {
        let suffix = suffix.trim_start_matches('/');
        handle_relay_set(topic, suffix, payload, relay_tx).await;
        return;
    }
    warn!("mqtt[sub]: ignoring unknown topic '{}'", topic);
}

async fn handle_relay_set(full_topic: &str, suffix: &str, payload: &[u8], relay_tx: &RelayTx) {
    let mut parts = suffix.split('/');
    let (idx_str, cmd, rest) = (parts.next(), parts.next(), parts.next());

    let idx_str = match (idx_str, cmd, rest) {
        (Some(i), Some("set"), None) => i,
        _ => {
            warn!("mqtt[sub]: malformed relay topic '{}'", full_topic);
            return;
        }
    };

    let idx = match u8::from_str(idx_str) {
        Ok(i) if (i as usize) < crate::bsp::relay::COUNT => i,
        Ok(i)  => { warn!("mqtt[sub]: relay index {} out of range (max {})", i, crate::bsp::relay::COUNT - 1); return; }
        Err(_) => { warn!("mqtt[sub]: non-numeric relay index '{}'", idx_str); return; }
    };

    let on = match payload {
        b"1" | b"on" | b"ON" | b"true"  | b"TRUE"  => true,
        b"0" | b"off" | b"OFF" | b"false" | b"FALSE" => false,
        other => {
            warn!("mqtt[sub]: unrecognised payload {:?} for relay {} — ignored", other, idx);
            return;
        }
    };

    info!("mqtt[sub]: relay {} -> {}", idx, if on { "on" } else { "off" });
    relay_tx.send(RelayCommand::Set { index: idx, on }).await;
}

async fn resolve(stack: &NetStack, host: &str) -> Option<embassy_net::IpAddress> {
    // Literal v4 fast-path — works without a DNS server.
    if let Ok(ip) = host.parse::<core::net::Ipv4Addr>() {
        return Some(embassy_net::IpAddress::v4(
            ip.octets()[0], ip.octets()[1], ip.octets()[2], ip.octets()[3],
        ));
    }
    match stack.dns_query(host, DnsQueryType::A).await {
        Ok(v) if !v.is_empty() => Some(v[0]),
        _ => None,
    }
}

fn write_f32(buf: &mut String<16>, v: f32) -> core::fmt::Result {
    use core::fmt::Write;
    write!(buf, "{:.2}", v)
}

// Silence the unused-CLIENT warning — kept for future single-client variants.
#[allow(dead_code)]
const _CLIENT_REF: &str = CLIENT;
