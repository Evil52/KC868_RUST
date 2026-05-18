//! Minimal async PCF8574 driver.
//!
//! The PCF8574 is a single 8-bit quasi-bidirectional I/O port. Reads return
//! the actual pin states (so it doubles as an input expander); writes drive
//! the open-drain outputs. We hold the desired output byte in software so
//! single-bit updates don't disturb the rest of the port.

use embedded_hal_async::i2c::I2c;

pub struct Pcf8574<BUS> {
    bus: BUS,
    addr: u8,
    /// Last byte we drove on the port. The PCF8574 has no read-back of the
    /// output latch — we have to mirror it ourselves.
    shadow: u8,
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self { Error::Bus(e) }
}

impl<BUS, E> Pcf8574<BUS>
where
    BUS: I2c<Error = E>,
{
    pub fn new(bus: BUS, addr: u8, initial: u8) -> Self {
        Self { bus, addr, shadow: initial }
    }

    /// Push the shadow byte to the chip.
    pub async fn commit(&mut self) -> Result<(), Error<E>> {
        self.bus.write(self.addr, &[self.shadow]).await?;
        Ok(())
    }

    /// Drive an explicit byte and remember it.
    pub async fn write_port(&mut self, value: u8) -> Result<(), Error<E>> {
        self.shadow = value;
        self.commit().await
    }

    /// Update a single bit in the shadow and commit.
    pub async fn set_bit(&mut self, bit: u8, high: bool) -> Result<(), Error<E>> {
        let mask = 1u8 << bit;
        let new = if high { self.shadow | mask } else { self.shadow & !mask };
        if new != self.shadow {
            self.shadow = new;
            self.commit().await?;
        }
        Ok(())
    }

    /// Read the physical port. For input use, drive the port high first
    /// (open-drain → external input pulls low when active).
    #[allow(dead_code)] // reserved for the input-expander (U6) task
    pub async fn read_port(&mut self) -> Result<u8, Error<E>> {
        let mut buf = [0u8; 1];
        self.bus.read(self.addr, &mut buf).await?;
        Ok(buf[0])
    }

    pub fn shadow(&self) -> u8 { self.shadow }
}
