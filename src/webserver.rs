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
        let mut rx_buf = [0u8; 4096];
        let mut tx_buf = [0u8; 4096];
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        // Drop a wedged client instead of hogging the (single) socket.
        socket.set_timeout(Some(Duration::from_secs(15)));

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
    let html = b"<!DOCTYPE html><html><head><meta charset=utf-8><meta name=viewport content=width=device-width><title>KC868-A6</title><style>*{margin:0;padding:0;box-sizing:border-box}body{font-family:ui-sans-serif,sans-serif;background:#02030d;color:#e7ecff;padding:20px;min-height:100vh}.navbar{background:linear-gradient(135deg,#0f1626,#1a2a3a);padding:1.5rem;margin:-20px -20px 20px;border-bottom:1px solid #3858b0;display:flex;justify-content:space-between;align-items:center}.brand{font-size:1.5rem;font-weight:700;color:#e7ecff;letter-spacing:1px}.container{max-width:1200px;margin:0 auto}.status-bar{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:1rem;margin-bottom:2rem}.status-card{background:#0f1626;border:1px solid #3858b0;border-radius:0.75rem;padding:1.5rem;text-align:center;transition:all 0.3s}.status-card:hover{border-color:#7aa7ff;box-shadow:0 0 16px rgba(122,167,255,0.2)}.status-label{font-size:0.75rem;color:#8a96c6;text-transform:uppercase;letter-spacing:2px;margin-bottom:0.5rem}.status-value{font-size:1.5rem;font-weight:700;color:#e7ecff}.relays-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:1.5rem}.relay-card{background:#0f1626;border:2px solid #3858b0;border-radius:0.75rem;padding:1.5rem;transition:all 0.3s;position:relative}.relay-card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;background:linear-gradient(90deg,transparent,#7aa7ff,transparent);opacity:0;transition:opacity 0.3s}.relay-card.on{border-color:#7aa7ff;box-shadow:0 0 16px rgba(122,167,255,0.15)}.relay-card.on::before{opacity:1}.relay-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:1rem}.relay-name{font-size:1rem;font-weight:600;color:#e7ecff}.relay-state{font-size:0.75rem;font-weight:700;padding:0.25rem 0.75rem;border-radius:4px;background:#3858b0;color:#e7ecff}.relay-state.on{background:#7aa7ff;color:#02030d}.relay-controls{display:flex;gap:0.75rem}.relay-btn{flex:1;padding:0.75rem;border:1px solid #3858b0;background:transparent;color:#e7ecff;border-radius:0.5rem;font-weight:600;cursor:pointer;transition:all 0.2s}.relay-btn:hover{border-color:#7aa7ff;background:rgba(122,167,255,0.1)}.relay-btn.active{background:#7aa7ff;border-color:#7aa7ff;color:#02030d}.indicator{display:inline-block;width:10px;height:10px;border-radius:50%;margin-left:0.5rem;animation:pulse 1.5s infinite}.indicator.ok{background:#7aa7ff}.indicator.locked{background:#ff6b6b;animation:pulse-danger 0.6s infinite}@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.3}}@keyframes pulse-danger{0%,100%{opacity:1}50%{opacity:0.4}}.footer{text-align:center;padding:2rem;color:#8a96c6;border-top:1px solid #3858b0;margin-top:2rem;font-size:0.875rem}@media(max-width:768px){.relays-grid{grid-template-columns:1fr}}</style></head><body><div class=navbar><div class=brand>* KC868-A6</div><div id=status style=font-size:0.875rem>[ON]</div></div><div class=container><div class=status-bar><div class=status-card><div class=status-label>Safety</div><div class=status-value><span id=safety>OK</span><span class=indicator id=safety-ind></span></div></div><div class=status-card><div class=status-label>Active</div><div class=status-value><span id=active>0</span>/6</div></div><div class=status-card><div class=status-label>Time</div><div class=status-value id=time style=font-size:1rem>--:--</div></div></div><div class=relays-grid id=relays></div><div class=status-card style=margin-top:2rem;text-align:left><div class=status-label>Firmware OTA</div><input type=file id=fw accept=.bin style=color:#e7ecff;margin:0.75rem 0;display:block><button class=relay-btn id=otabtn onclick=otaUpload() style=max-width:220px>Upload &amp; Flash</button><div id=otastat style=margin-top:0.75rem;color:#8a96c6;font-size:0.875rem></div></div><div class=footer>KC868-A6 Control Panel - Edge Relay System</div></div><script>const RELAY_COUNT=6;function init(){renderRelays();update();setInterval(update,1000)}function renderRelays(){const c=document.getElementById('relays');for(let i=0;i<RELAY_COUNT;i++){const d=document.createElement('div');d.className='relay-card';d.id='r'+i;d.innerHTML='<div class=relay-header><div class=relay-name>Relay '+(i+1)+'</div><div class=relay-state id=s'+i+'>OFF</div></div><div class=relay-controls><button class=relay-btn onclick=\"set('+i+',1)\">ON</button><button class=relay-btn onclick=\"set('+i+',0)\">OFF</button></div>';c.appendChild(d)}}function set(i,v){fetch('/api/relay/'+i+'/set',{method:'POST',body:JSON.stringify({state:v})}).then(()=>update()).catch(e=>console.log(e))}async function update(){try{const r=await fetch('/api/status');const d=await r.json();let a=0;for(let i=0;i<RELAY_COUNT;i++){const on=!!(d.mask&(1<<i));document.getElementById('s'+i).textContent=on?'ON':'OFF';document.getElementById('s'+i).classList.toggle('on',on);document.getElementById('r'+i).classList.toggle('on',on);if(on)a++}document.getElementById('active').textContent=a;document.getElementById('safety').textContent=d.locked?'LOCKED':'OK';document.getElementById('safety-ind').className='indicator '+(d.locked?'locked':'ok');const n=new Date();document.getElementById('time').textContent=String(n.getHours()).padStart(2,'0')+':'+String(n.getMinutes()).padStart(2,'0');}catch(e){}}function crc32(b){let t=crc32.t;if(!t){t=[];for(let n=0;n<256;n++){let c=n;for(let k=0;k<8;k++)c=c&1?0xEDB88320^(c>>>1):c>>>1;t[n]=c>>>0;}crc32.t=t;}let c=0xFFFFFFFF;for(let i=0;i<b.length;i++)c=t[(c^b[i])&255]^(c>>>8);return(c^0xFFFFFFFF)>>>0;}async function otaUpload(){const f=document.getElementById('fw').files[0];const s=document.getElementById('otastat');if(!f){s.textContent='Choose a .bin first';return;}const b=new Uint8Array(await f.arrayBuffer());const c=crc32(b);const btn=document.getElementById('otabtn');btn.disabled=true;s.textContent='Uploading '+b.length+' bytes, crc '+c+' ...';try{const r=await fetch('/api/ota',{method:'POST',headers:{'X-Ota-Crc32':String(c)},body:b});s.textContent=await r.text();}catch(e){s.textContent='Device rebooting (upload likely OK)';}btn.disabled=false;}document.addEventListener('DOMContentLoaded',init);</script></body></html>";

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
