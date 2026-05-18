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

use core::str::FromStr;

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

use crate::bsp::mqtt_topic;
use crate::relays::{RelayCommand, RelayTx};
use crate::wifi::NetStack;

const HOST:   &str = env!("MQTT_HOST");
const PORT:   &str = env!("MQTT_PORT");
const CLIENT: &str = env!("MQTT_CLIENT_ID");

const RECONNECT_DELAY: Duration = Duration::from_secs(2);
const TCP_TIMEOUT:     Duration = Duration::from_secs(20);
const TCP_KEEPALIVE:   Duration = Duration::from_secs(15);

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

        let mut client = MqttClient::<_, 5, _>::new(
            socket, &mut mqtt_tx, 1024, &mut mqtt_rx, 1024, cfg,
        );

        if let Err(e) = client.connect_to_broker().await {
            warn!("mqtt[pub]: connect failed: {:?}", e);
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }
        info!("mqtt[pub]: connected");

        let _ = client
            .send_message(mqtt_topic::STATUS, b"online", QualityOfService::QoS0, true)
            .await;

        // Inner loop — publish every signalled temperature.
        let res = loop {
            let t = TEMPERATURE.wait().await;
            let mut payload: String<16> = String::new();
            let _ = write_f32(&mut payload, t);
            if let Err(e) = client
                .send_message(mqtt_topic::TEMPERATURE, payload.as_bytes(),
                              QualityOfService::QoS0, false)
                .await
            {
                break e;
            }
        };

        warn!("mqtt[pub]: session ended: {:?}", res);
        Timer::after(RECONNECT_DELAY).await;
    }
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
        cfg.keep_alive = 30;

        let mut client = MqttClient::<_, 5, _>::new(
            socket, &mut mqtt_tx, 1024, &mut mqtt_rx, 1024, cfg,
        );

        if let Err(e) = client.connect_to_broker().await {
            warn!("mqtt[sub]: connect failed: {:?}", e);
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }

        if let Err(e) = client.subscribe_to_topic(mqtt_topic::RELAY_CMD_SUB).await {
            warn!("mqtt[sub]: subscribe failed: {:?}", e);
            Timer::after(RECONNECT_DELAY).await;
            continue;
        }
        info!("mqtt[sub]: subscribed to {}", mqtt_topic::RELAY_CMD_SUB);

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
    // Expect "kc868/relay/<N>/set"
    let suffix = match topic.strip_prefix(mqtt_topic::RELAY_STATE_PREFIX) {
        Some(s) => s.trim_start_matches('/'),
        None => return,
    };
    let mut parts = suffix.split('/');
    let (Some(idx_str), Some("set")) = (parts.next(), parts.next()) else { return };
    let Ok(idx) = u8::from_str(idx_str) else { return };
    let on = matches!(payload, b"1" | b"on" | b"ON" | b"true");
    info!("mqtt[sub]: relay {} -> {}", idx, on);
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
