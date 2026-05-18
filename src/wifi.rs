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
            let cfg = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
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
