//! Over-the-air firmware update for KC868-A6 (bare-metal, no esp-idf).
//!
//! Implements the ESP-IDF OTA partition scheme directly on top of
//! [`esp_storage::FlashStorage`]:
//!   1. parse the partition table at `0x8000` to locate `ota_0`/`ota_1`
//!      and `otadata`,
//!   2. pick the *inactive* slot (derived from `otadata` — the bootloader
//!      always boots the highest valid sequence, falling back to `ota_0`
//!      when `otadata` is blank),
//!   3. stream the new image into that slot,
//!   4. verify its CRC-32 by reading it back, then
//!   5. rewrite the inactive `otadata` copy with a higher sequence so the
//!      2nd-stage bootloader boots the new slot on next reset.
//!
//! We roll this ourselves rather than use `esp-hal-ota`: that crate's MMU
//! helper relies on the `concat_idents!` macro, which the current Xtensa
//! toolchain has removed. We don't need the MMU — `otadata` tells us the
//! running slot.
//!
//! Safety: the boot pointer moves only after the written slot's CRC-32 is
//! re-read and matches what the client advertised, and the bootloader
//! independently verifies the image SHA-256 on boot. A USB reflash (which
//! erases `otadata`) always recovers the device.
//!
//! See [`crate::webserver`] for the `POST /api/ota` streaming endpoint and
//! `tools/ota.ps1` for the host-side uploader.

use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;

/// Flash sector size. Feed [`OtaSession::write`] in multiples of this so
/// each sector is erased+written exactly once — `esp-storage` performs a
/// full read-modify-erase-write per call.
pub const SECTOR: usize = 4096;

const PT_OFFSET: u32 = 0x8000; // partition table base
const PT_SCAN: u32 = 0xc00; //    bytes of table to scan (96 entries)
const ENTRY: usize = 32; //       partition / otadata entry size
const MAGIC0: u8 = 0xAA;
const MAGIC1: u8 = 0x50;
const TYPE_APP: u8 = 0x00;
const TYPE_DATA: u8 = 0x01;
const SUBTYPE_OTA0: u8 = 0x10; // app, ota_0 (ota_1 = 0x11, ...)
const SUBTYPE_OTADATA: u8 = 0x00; // data, ota
const STATE_UNDEFINED: u32 = 0xFFFF_FFFF;

#[derive(Debug, Clone, Copy)]
pub enum OtaError {
    /// Partition table lacks two app slots + an otadata partition.
    NoOtaPartitions,
    /// Image is larger than the target slot.
    TooBig,
    /// A flash read/write failed.
    Flash,
    /// Streamed or read-back CRC-32 did not match the advertised value.
    Crc,
}

impl OtaError {
    pub fn as_str(self) -> &'static str {
        match self {
            OtaError::NoOtaPartitions => "no OTA partitions",
            OtaError::TooBig => "image too big for slot",
            OtaError::Flash => "flash error",
            OtaError::Crc => "CRC mismatch",
        }
    }
}

/// OTA slot + otadata geometry read from the partition table.
struct Layout {
    ota: [(u32, u32); 2], // (offset, size) for ota_0, ota_1
    count: usize,
    otadata_off: u32,
    otadata_size: u32,
}

/// An in-progress write into the inactive OTA slot.
pub struct OtaSession {
    flash: FlashStorage,
    layout: Layout,
    seq0: u32, // sequence in otadata copy 0 (0 = invalid/blank)
    seq1: u32, // sequence in otadata copy 1
    target: usize,
    slot_off: u32,
    size: u32,
    written: u32,
    target_crc: u32,
    run_crc: u32,
}

impl OtaSession {
    /// Open a session for an image of `size` bytes whose IEEE CRC-32 is
    /// `crc`. Selects the slot the bootloader is *not* currently running.
    pub fn begin(size: u32, crc: u32) -> Result<Self, OtaError> {
        let mut flash = FlashStorage::new();
        let layout = read_layout(&mut flash)?;

        let half = layout.otadata_size / 2;
        let seq0 = read_seq(&mut flash, layout.otadata_off);
        let seq1 = read_seq(&mut flash, layout.otadata_off + half);

        let max_seq = seq0.max(seq1);
        let current = if max_seq == 0 {
            0 // blank otadata → bootloader runs ota_0
        } else {
            seq_to_part(max_seq, layout.count)
        };
        let target = (current + 1) % layout.count;
        let (slot_off, slot_size) = layout.ota[target];

        if size > slot_size {
            return Err(OtaError::TooBig);
        }

        Ok(Self {
            flash,
            layout,
            seq0,
            seq1,
            target,
            slot_off,
            size,
            written: 0,
            target_crc: crc,
            run_crc: 0,
        })
    }

    /// Write the next chunk into the slot. Returns `Ok(true)` once the
    /// advertised size has been fully written.
    pub fn write(&mut self, chunk: &[u8]) -> Result<bool, OtaError> {
        let remaining = self.size - self.written;
        let n = core::cmp::min(chunk.len() as u32, remaining) as usize;
        if n == 0 {
            return Ok(true);
        }
        self.flash
            .write(self.slot_off + self.written, &chunk[..n])
            .map_err(|_| OtaError::Flash)?;
        self.run_crc = crc32(&chunk[..n], self.run_crc);
        self.written += n as u32;
        Ok(self.written >= self.size)
    }

    /// Verify the written slot, then commit it as the next boot target.
    pub fn finish(&mut self) -> Result<(), OtaError> {
        if self.run_crc != self.target_crc {
            return Err(OtaError::Crc);
        }
        if self.verify()? != self.target_crc {
            return Err(OtaError::Crc);
        }

        // Next sequence that maps to the target slot (and is non-zero).
        let mut seq = self.seq0.max(self.seq1);
        loop {
            seq += 1;
            if seq != 0 && seq_to_part(seq, self.layout.count) == self.target {
                break;
            }
        }

        let mut entry = [0xFFu8; ENTRY];
        entry[0..4].copy_from_slice(&seq.to_le_bytes());
        entry[24..28].copy_from_slice(&STATE_UNDEFINED.to_le_bytes());
        entry[28..32].copy_from_slice(&crc32(&seq.to_le_bytes(), 0xFFFF_FFFF).to_le_bytes());

        // Write into the otadata copy that currently holds the *lower*
        // sequence, so the freshly written (higher) one wins next boot.
        let half = self.layout.otadata_size / 2;
        let dst = if self.seq0 > self.seq1 {
            self.layout.otadata_off + half
        } else {
            self.layout.otadata_off
        };
        self.flash.write(dst, &entry).map_err(|_| OtaError::Flash)?;
        Ok(())
    }

    /// Re-read the written slot and return its CRC-32.
    fn verify(&mut self) -> Result<u32, OtaError> {
        let mut crc = 0u32;
        let mut buf = [0u8; 256];
        let mut off = self.slot_off;
        let mut rem = self.size;
        while rem > 0 {
            let n = core::cmp::min(rem, buf.len() as u32) as usize;
            self.flash
                .read(off, &mut buf[..n])
                .map_err(|_| OtaError::Flash)?;
            crc = crc32(&buf[..n], crc);
            off += n as u32;
            rem -= n as u32;
        }
        Ok(crc)
    }
}

/// Reboot into the freshly committed slot. Never returns.
pub fn reboot() -> ! {
    esp_hal::reset::software_reset();
    // `software_reset()` returns `()` on esp-hal 0.23; the reset lands
    // before the next instruction, so this spin only satisfies `-> !`.
    loop {}
}

/// ESP-IDF sequence-number → app-partition index.
#[inline]
fn seq_to_part(seq: u32, count: usize) -> usize {
    (seq.saturating_sub(1) as usize) % count
}

/// Read an otadata copy and return its sequence, or 0 if the embedded
/// CRC of the sequence does not check out (blank/invalid copy).
fn read_seq(flash: &mut FlashStorage, off: u32) -> u32 {
    let mut e = [0u8; ENTRY];
    if flash.read(off, &mut e).is_err() {
        return 0;
    }
    let seq = u32::from_le_bytes([e[0], e[1], e[2], e[3]]);
    let crc = u32::from_le_bytes([e[28], e[29], e[30], e[31]]);
    if crc32(&seq.to_le_bytes(), 0xFFFF_FFFF) == crc {
        seq
    } else {
        0
    }
}

/// Scan the partition table for the OTA slots + otadata.
fn read_layout(flash: &mut FlashStorage) -> Result<Layout, OtaError> {
    let mut layout = Layout {
        ota: [(0, 0); 2],
        count: 0,
        otadata_off: 0,
        otadata_size: 0,
    };

    let mut buf = [0u8; ENTRY];
    let mut off = 0u32;
    while off < PT_SCAN {
        if flash.read(PT_OFFSET + off, &mut buf).is_err() {
            break;
        }
        off += ENTRY as u32;

        if buf == [0xFFu8; ENTRY] {
            break; // end of table
        }
        if buf[0] != MAGIC0 || buf[1] != MAGIC1 {
            continue; // not a partition entry (e.g. the md5 checksum row)
        }

        let ptype = buf[2];
        let psub = buf[3];
        let poff = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        let psize = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);

        if ptype == TYPE_APP && psub >= SUBTYPE_OTA0 {
            let idx = (psub - SUBTYPE_OTA0) as usize;
            if idx < layout.ota.len() {
                layout.ota[idx] = (poff, psize);
                if idx + 1 > layout.count {
                    layout.count = idx + 1;
                }
            }
        } else if ptype == TYPE_DATA && psub == SUBTYPE_OTADATA {
            layout.otadata_off = poff;
            layout.otadata_size = psize;
        }
    }

    if layout.count < 2 || layout.otadata_size == 0 {
        return Err(OtaError::NoOtaPartitions);
    }
    Ok(layout)
}

/// IEEE CRC-32 (zlib/PNG), table-driven. Matches the host-side uploader
/// and the bootloader's otadata sequence CRC. `crc` is the running value
/// (seed `0` for a fresh image, `0xFFFF_FFFF` for the otadata sequence).
fn crc32(buf: &[u8], crc: u32) -> u32 {
    let mut c = !crc;
    for &b in buf {
        c = CRC_TAB[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    c ^ 0xFFFF_FFFF
}

#[rustfmt::skip]
const CRC_TAB: [u32; 256] = [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
];
