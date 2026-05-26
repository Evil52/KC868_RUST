//! WiFi station bring-up + embassy-net stack runner.
//!
//! Two long-running tasks:
//!   * `connection_task` — owns the WiFi controller, (re)connects on drop.
//!   * `net_task`        — pumps the embassy-net stack.

use embassy_executor::Spawner;
use embassy_net::{Config, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_wifi::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
    WifiState,
};
use log::{info, warn};
use static_cell::StaticCell;

const SSID: &str = env!("ESP32");
const PASSWORD: &str = env!("14310324");

// Compile-time bounds: ESP32 caps SSID at 32 bytes and password at 64
// (heapless capacity of `ClientConfiguration::{ssid, password}`). If
// these asserts ever fire, the build fails — no runtime panic path.
const _: () = {
    assert!(SSID.len() <= 32, "WIFI SSID > 32 bytes — see .cargo/config.toml");
    assert!(PASSWORD.len() <= 64, "WIFI PASSWORD > 64 bytes — see .cargo/config.toml");
};

pub type NetStack = Stack<'static>;

pub fn start(
    spawner: &Spawner,
    controller: WifiController<'static>,
    device: WifiDevice<'static, WifiStaDevice>,
    seed: u64,
) -> NetStack {
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let resources = RESOURCES.init(StackResources::<4>::new());

    let (stack, runner) =
        embassy_net::new(device, Config::dhcpv4(Default::default()), resources, seed);

    spawner.must_spawn(connection_task(controller));
    spawner.must_spawn(net_task(runner));
    stack
}

#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
    info!("wifi: starting (SSID={})", SSID);
    loop {
        if esp_wifi::wifi::wifi_state() == WifiState::StaConnected {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            warn!("wifi: disconnected, retrying in 2s");
            Timer::after(Duration::from_secs(2)).await;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            // Both `try_into`s are infallible at runtime — the const
            // asserts above prove SSID ≤ 32 and PASSWORD ≤ 64 bytes.
            // We still match on the `Result` so the compiler can verify
            // there's no panic path. Hitting the `Err` arm would mean
            // a heapless API change we missed.
            let ssid = match SSID.try_into() {
                Ok(s) => s,
                Err(_) => {
                    warn!("wifi: SSID rejected by heapless (should be impossible)");
                    Timer::after(Duration::from_secs(60)).await;
                    continue;
                }
            };
            let password = match PASSWORD.try_into() {
                Ok(p) => p,
                Err(_) => {
                    warn!("wifi: password rejected by heapless (should be impossible)");
                    Timer::after(Duration::from_secs(60)).await;
                    continue;
                }
            };
            let cfg = Configuration::Client(ClientConfiguration {
                ssid,
                password,
                ..Default::default()
            });
            if let Err(e) = controller.set_configuration(&cfg) {
                warn!("wifi: set_configuration failed: {:?}", e);
                continue;
            }
            if let Err(e) = controller.start_async().await {
                warn!("wifi: start failed: {:?}", e);
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        }

        match controller.connect_async().await {
            Ok(()) => info!("wifi: associated"),
            Err(e) => {
                warn!("wifi: connect failed: {:?}", e);
                Timer::after(Duration::from_secs(2)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    runner.run().await;
}

/// Block until DHCP has handed us an address.
pub async fn wait_for_link(stack: &NetStack) {
    loop {
        if let Some(cfg) = stack.config_v4() {
            info!("net: link up, ip = {}", cfg.address);
            return;
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}
