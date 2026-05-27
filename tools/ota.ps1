<#
.SYNOPSIS
  Build (optional), package, and flash a KC868-A6 firmware image over WiFi.

.DESCRIPTION
  Produces the ESP32 application image with `espflash save-image`, computes
  its IEEE CRC-32, and POSTs it to the device's /api/ota endpoint. The device
  writes it into the *inactive* OTA slot, re-reads + verifies the CRC, flips
  the boot pointer in `otadata`, then reboots into the new firmware.

  No USB cable needed — the device only has to be powered and on the network.
  (The one-time bootstrap that installs the OTA partition table must still be
  done over USB; see the project README / chat notes.)

.EXAMPLE
  ./tools/ota.ps1 -Build
  ./tools/ota.ps1 -Device 192.168.137.244
#>
[CmdletBinding()]
param(
    [string]$Device     = "192.168.137.244",
    [string]$Elf        = "target/xtensa-esp32-none-elf/release/kc868_a6",
    [string]$Bin        = "target/xtensa-esp32-none-elf/release/kc868_a6.ota.bin",
    [switch]$Build,
    [int]$TimeoutSec    = 180
)

$ErrorActionPreference = 'Stop'

# IEEE CRC-32 (zlib/PNG) — must match src/ota.rs and the in-browser uploader.
# NOTE: avoid 8-digit hex literals (0xFFFFFFFF / 0xEDB88320) — Windows
# PowerShell 5.1 parses them as negative Int32, which breaks [uint32] casts.
function Get-Crc32 {
    param([byte[]]$Data)
    $poly = [uint32]3988292384   # 0xEDB88320 (reversed polynomial)
    $table = New-Object 'System.UInt32[]' 256
    for ($n = 0; $n -lt 256; $n++) {
        $c = [uint32]$n
        for ($k = 0; $k -lt 8; $k++) {
            if (($c -band 1) -ne 0) { $c = [uint32](($c -shr 1) -bxor $poly) }
            else                    { $c = [uint32]($c -shr 1) }
        }
        $table[$n] = $c
    }
    $crc = [uint32]::MaxValue
    foreach ($b in $Data) {
        $idx = [int](($crc -bxor [uint32]$b) -band 0xFF)
        $crc = [uint32]($table[$idx] -bxor ($crc -shr 8))
    }
    return [uint32]($crc -bxor [uint32]::MaxValue)
}

if ($Build) {
    Write-Host "==> cargo build --release --bin kc868_a6" -ForegroundColor Cyan
    cargo build --release --bin kc868_a6
    if ($LASTEXITCODE -ne 0) { throw "cargo build failed" }
}

if (-not (Test-Path $Elf)) { throw "ELF not found: $Elf  (run with -Build, or pass -Elf)" }

Write-Host "==> espflash save-image -> $Bin" -ForegroundColor Cyan
espflash save-image --chip esp32 --flash-size 4mb --ignore-app-descriptor $Elf $Bin
if ($LASTEXITCODE -ne 0) { throw "espflash save-image failed" }

$bytes = [System.IO.File]::ReadAllBytes((Resolve-Path $Bin))
$crc   = Get-Crc32 -Data $bytes
Write-Host ("==> image: {0} bytes, crc32 {1}" -f $bytes.Length, $crc) -ForegroundColor Cyan

$uri = "http://$Device/api/ota"
Write-Host "==> POST $uri  (device flashes + verifies + reboots, ~15s)..." -ForegroundColor Cyan

Add-Type -AssemblyName System.Net.Http
$client = New-Object System.Net.Http.HttpClient
$client.Timeout = [TimeSpan]::FromSeconds($TimeoutSec)
try {
    $req = New-Object System.Net.Http.HttpRequestMessage([System.Net.Http.HttpMethod]::Post, $uri)
    $req.Content = New-Object System.Net.Http.ByteArrayContent (,$bytes)
    $req.Content.Headers.ContentType =
        [System.Net.Http.Headers.MediaTypeHeaderValue]::new('application/octet-stream')
    [void]$req.Headers.TryAddWithoutValidation('X-Ota-Crc32', "$crc")

    $resp = $client.SendAsync($req).GetAwaiter().GetResult()
    $body = $resp.Content.ReadAsStringAsync().GetAwaiter().GetResult()
    Write-Host ("==> device replied [{0}]: {1}" -f [int]$resp.StatusCode, $body) -ForegroundColor Green
} catch {
    Write-Host "==> connection ended (device likely rebooted into new firmware): $($_.Exception.Message)" -ForegroundColor Yellow
} finally {
    $client.Dispose()
}
