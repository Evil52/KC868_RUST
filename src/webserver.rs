//! Production HTTP web server for KC868-A6 relay control.
//!
//! Serves on port 80:
//!   GET  /              — Full HTML dashboard with real-time updates
//!   GET  /api/status    — JSON with relay states and safety status
//!   POST /api/relay/N   — Set relay N (body: {"state":0|1})

use core::fmt::Write as _;
use core::str::FromStr;
use embedded_io_async::Write;
use embassy_net::{tcp::TcpSocket, Stack};
use embassy_time::{Duration, Timer};
use heapless::String;
use log::{info, warn};

use crate::ota;
use crate::relays::{self, RelayCommand, RelayTx};
use crate::safety;

const HTTP_PORT: u16 = 80;

#[embassy_executor::task]
pub async fn webserver_task(stack: Stack<'static>, relay_tx: RelayTx) {
    info!("webserver: starting on port {}", HTTP_PORT);

    // One fresh socket per connection. Reusing a single socket across
    // accept/close cycles is an embassy-net footgun — a closed socket
    // won't re-`accept` cleanly and ends up RST-ing clients.
    loop {
        let mut rx_buf = [0u8; 8192];
        let mut tx_buf = [0u8; 4096];
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        // Drop a wedged client instead of hogging the (single) socket.
        // OTA uploads can take ~30 s (flash write) so keep timeout generous.
        socket.set_timeout(Some(Duration::from_secs(120)));

        if socket.accept(HTTP_PORT).await.is_err() {
            warn!("webserver: accept failed, retrying");
            Timer::after(Duration::from_millis(200)).await;
            continue;
        }

        // Read the first segment (request line + headers, maybe some body).
        let mut buf = [0u8; 4096];
        let mut reboot = false;
        match socket.read(&mut buf).await {
            Ok(n) if n > 0 => {
                reboot = dispatch(&buf[..n], &mut socket, &relay_tx).await;
                // Push the response out before we tear the connection down.
                let _ = socket.flush().await;
            }
            _ => {}
        }

        socket.close();
        if reboot {
            // OTA committed — give the client time to read the reply and
            // the FIN to flush, then boot the freshly written slot.
            Timer::after(Duration::from_millis(1200)).await;
            ota::reboot();
        }
        // Let the FIN drain before the buffers are reused next iteration.
        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Index just past the `\r\n\r\n` that ends the HTTP header block.
fn headers_end(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| w == b"\r\n\r\n").map(|p| p + 4)
}

/// Route one request. Returns `true` iff an OTA image was committed and
/// the caller should reboot into the new slot.
async fn dispatch(req: &[u8], socket: &mut TcpSocket<'_>, relay_tx: &RelayTx) -> bool {
    let head_end = match headers_end(req) {
        Some(e) => e,
        None => return false, // header block not fully in the first segment
    };
    // Request line + headers are ASCII; the body may be a binary firmware
    // image, so only decode the head.
    let head = match core::str::from_utf8(&req[..head_end]) {
        Ok(s) => s,
        Err(_) => return false,
    };

    let request_line = head.lines().next().unwrap_or("");
    let mut fields = request_line.split(' ');
    let method = fields.next().unwrap_or("");
    let path = fields.next().unwrap_or("");

    match (method, path) {
        ("GET", "/") => {
            send_html(socket).await;
        }
        ("GET", "/api/status") => {
            send_json_status(socket).await;
        }
        ("POST", "/api/ota") => {
            // Headers carry the image length (Content-Length) and its
            // IEEE CRC-32 (X-Ota-Crc32, decimal).
            let mut size = 0u32;
            let mut crc = 0u32;
            for line in head.lines().skip(1) {
                if let Some((name, val)) = line.split_once(':') {
                    let (name, val) = (name.trim(), val.trim());
                    if name.eq_ignore_ascii_case("content-length") {
                        size = val.parse().unwrap_or(0);
                    } else if name.eq_ignore_ascii_case("x-ota-crc32") {
                        crc = val.parse().unwrap_or(0);
                    }
                }
            }
            return handle_ota(socket, &req[head_end..], size, crc).await;
        }
        ("POST", p) if p.starts_with("/api/relay/") => {
            handle_relay(p, &req[head_end..], relay_tx, socket).await;
        }
        _ => {}
    }
    false
}

async fn handle_relay(
    path: &str,
    body: &[u8],
    relay_tx: &RelayTx,
    socket: &mut TcpSocket<'_>,
) {
    let suffix = path.trim_start_matches("/api/relay/");
    if let Some(idx_str) = suffix.strip_suffix("/set") {
        if let Ok(idx) = u8::from_str(idx_str) {
            if let Ok(body) = core::str::from_utf8(body) {
                if body.contains("\"state\":1") {
                    relay_tx.send(RelayCommand::Set { index: idx, on: true }).await;
                } else if body.contains("\"state\":0") {
                    relay_tx.send(RelayCommand::Set { index: idx, on: false }).await;
                }
            }
            send_ok(socket).await;
        }
    }
}

/// Stream a firmware image into the inactive OTA slot. `body0` is the
/// slice of image bytes that already arrived in the header segment.
/// Returns `true` once the image is committed and a reboot is due.
async fn handle_ota(socket: &mut TcpSocket<'_>, body0: &[u8], size: u32, crc: u32) -> bool {
    if size == 0 {
        warn!("ota: missing/zero Content-Length");
        send_ota_result(socket, false, "missing Content-Length").await;
        return false;
    }
    info!("ota: begin size={} crc={}", size, crc);

    let mut session = match ota::OtaSession::begin(size, crc) {
        Ok(s) => s,
        Err(e) => {
            warn!("ota: begin failed: {}", e.as_str());
            send_ota_result(socket, false, e.as_str()).await;
            return false;
        }
    };

    // Accumulate into whole flash sectors so each is erased+written once.
    let mut sector = [0u8; ota::SECTOR];
    let seed = core::cmp::min(body0.len(), size as usize);
    sector[..seed].copy_from_slice(&body0[..seed]);
    let mut fill = seed;
    let mut received = seed as u32;

    while received < size {
        let want = core::cmp::min(ota::SECTOR - fill, (size - received) as usize);
        let n = match socket.read(&mut sector[fill..fill + want]).await {
            Ok(0) => {
                warn!("ota: connection closed at {}/{}", received, size);
                send_ota_result(socket, false, "short upload").await;
                return false;
            }
            Ok(n) => n,
            Err(_) => {
                warn!("ota: socket read error");
                return false;
            }
        };
        fill += n;
        received += n as u32;
        if fill == ota::SECTOR {
            if let Err(e) = session.write(&sector) {
                warn!("ota: write failed: {}", e.as_str());
                send_ota_result(socket, false, e.as_str()).await;
                return false;
            }
            fill = 0;
        }
    }

    // Flush the trailing partial sector, if any.
    if fill > 0 {
        if let Err(e) = session.write(&sector[..fill]) {
            warn!("ota: final write failed: {}", e.as_str());
            send_ota_result(socket, false, e.as_str()).await;
            return false;
        }
    }

    match session.finish() {
        Ok(()) => {
            info!("ota: committed {} bytes, rebooting", size);
            send_ota_result(socket, true, "OK - rebooting into new firmware").await;
            true
        }
        Err(e) => {
            warn!("ota: finish failed: {}", e.as_str());
            send_ota_result(socket, false, e.as_str()).await;
            false
        }
    }
}

async fn send_html(socket: &mut TcpSocket<'_>) {
    let html = include_bytes!("dashboard.html");

    let mut resp: String<160> = String::new();
    let _ = write!(
        resp,
        "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        html.len()
    );

    let _ = socket.write_all(resp.as_bytes()).await;
    let _ = socket.write_all(html).await;
}

async fn send_json_status(socket: &mut TcpSocket<'_>) {
    let on_mask = relays::current_on_mask();
    let locked = safety::is_locked();

    let mut json: String<256> = String::new();
    let _ = write!(
        json,
        "{{\"mask\":{},\"locked\":{}}}",
        on_mask,
        if locked { "true" } else { "false" }
    );

    let mut resp: String<128> = String::new();
    let _ = write!(
        resp,
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        json.len()
    );

    let _ = socket.write_all(resp.as_bytes()).await;
    let _ = socket.write_all(json.as_bytes()).await;
}

async fn send_ok(socket: &mut TcpSocket<'_>) {
    let _ = socket
        .write_all(b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}")
        .await;
}

async fn send_ota_result(socket: &mut TcpSocket<'_>, ok: bool, msg: &str) {
    let mut body: String<128> = String::new();
    let _ = write!(
        body,
        "{{\"ok\":{},\"msg\":\"{}\"}}",
        if ok { "true" } else { "false" },
        msg
    );

    let mut resp: String<160> = String::new();
    let _ = write!(
        resp,
        "HTTP/1.1 {}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        if ok { "200 OK" } else { "400 Bad Request" },
        body.len()
    );

    let _ = socket.write_all(resp.as_bytes()).await;
    let _ = socket.write_all(body.as_bytes()).await;
}
