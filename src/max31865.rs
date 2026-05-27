//! MAX31865 RTD-to-Digital converter (Adafruit/generic breakout).
//!
//! 2-wire Pt100 configuration:
//!   * Solder/wire jumpers between RTD+/RTDIN+ and RTD-/RTDIN- on the board.
//!   * Config register = 0xC3 (50 Hz mains regions; use 0xC2 for 60 Hz):
//!       bit 7  VBIAS              = 1 (on)
//!       bit 6  Conversion mode    = 1 (auto, continuous)
//!       bit 5  1-shot             = 0
//!       bit 4  3-wire             = 0  (2-wire / 4-wire)
//!       bits 3-2 Fault detection  = 00
//!       bit 1  Fault clear        = 1
//!       bit 0  50 Hz filter       = 1 (clear to 0 in 60 Hz mains regions)
//!
//! Register conversion (datasheet §8):
//!     R_rtd = (raw >> 1) * Rref / 32768
//! where `raw` is the 16-bit value read from RTD_MSB+RTD_LSB. The LSB of
//! that 16-bit word is the fault flag, so we right-shift one bit before
//! computing.

use embedded_hal_async::spi::SpiDevice;

#[allow(dead_code)] // documents the read counterpart of REG_CONFIG_WRITE
const REG_CONFIG_READ:  u8 = 0x00;
const REG_CONFIG_WRITE: u8 = 0x80;
const REG_RTD_MSB:      u8 = 0x01;
const REG_FAULT_STATUS: u8 = 0x07;

const CONFIG_2WIRE_AUTO_50HZ: u8 = 0xC3;

#[derive(Debug)]
pub enum Error<E> {
    Spi(E),
    /// MAX31865 raised a fault flag (open circuit, short, over/under-voltage).
    Fault(u8),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self { Error::Spi(e) }
}

pub struct Max31865<SPI> {
    spi: SPI,
}

impl<SPI, E> Max31865<SPI>
where
    SPI: SpiDevice<u8, Error = E>,
{
    pub fn new(spi: SPI) -> Self { Self { spi } }

    /// Bring up the chip in 2-wire continuous mode.
    pub async fn init(&mut self) -> Result<(), Error<E>> {
        self.write_reg(REG_CONFIG_WRITE, CONFIG_2WIRE_AUTO_50HZ).await
    }

    /// Read the latest raw 15-bit RTD code (fault bit already stripped).
    pub async fn read_raw(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [REG_RTD_MSB, 0, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        let raw = (u16::from(buf[1]) << 8) | u16::from(buf[2]);
        if raw & 0x0001 != 0 {
            let fault = self.read_reg(REG_FAULT_STATUS).await?;
            // Best effort: clear the fault flag so we'll retry next sample.
            let _ = self.write_reg(REG_CONFIG_WRITE, CONFIG_2WIRE_AUTO_50HZ).await;
            return Err(Error::Fault(fault));
        }
        Ok(raw >> 1)
    }

    /// Read RTD resistance in ohms.
    pub async fn read_resistance(&mut self, rref_ohms: f32) -> Result<f32, Error<E>> {
        let code = self.read_raw().await?;
        Ok(f32::from(code) * rref_ohms / 32_768.0)
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error<E>> {
        self.spi.write(&[reg, value]).await?;
        Ok(())
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut buf = [reg & 0x7F, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
    }
}
