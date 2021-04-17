use embedded_hal::blocking::i2c;
#[cfg(feature = "board-rev-3")]
use lm75::{Error as Lm75Error, Lm75, SlaveAddr};
use nb::Error as NbError;
use stm32f1xx_hal::gpio::gpiob::{PB10, PB11};
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
use stm32f1xx_hal::i2c::{BlockingI2c, Error as I2cError};

pub struct I2cBus {
    #[allow(unused)]
    #[cfg(all(not(feature = "board-rev-2"), feature = "i2c2"))]
    pub(super) i2c: BlockingI2c<
        stm32f1xx_hal::pac::I2C2,
        (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>),
    >,
}

impl I2cBus {
    #[cfg(feature = "board-rev-3")]
    pub fn read_temperature(&mut self) -> Result<f32, Lm75Error<NbError<I2cError>>> {
        Lm75::new(I2cRef(&mut self.i2c), SlaveAddr::Default).read_temperature()
    }

    #[cfg(feature = "board-rev-3")]
    pub fn read_temperature_blocking(&mut self) -> Result<f32, Lm75Error<I2cError>> {
        loop {
            match self.read_temperature() {
                Ok(v) => break Ok(v),
                Err(lm75::Error::InvalidInputData) => break Err(lm75::Error::InvalidInputData),
                Err(lm75::Error::I2C(nb::Error::Other(e))) => break Err(lm75::Error::I2C(e)),
                Err(lm75::Error::I2C(nb::Error::WouldBlock)) => continue,
            }
        }
    }
}

pub struct I2cRef<'a, I: i2c::Write>(&'a mut I);

impl<I: i2c::Write> i2c::Write for I2cRef<'_, I> {
    type Error = I::Error;

    #[inline]
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.write(addr, bytes)
    }
}

impl<I: i2c::Write> i2c::Read for I2cRef<'_, I>
where
    I: i2c::Read,
{
    type Error = <I as i2c::Read>::Error;

    #[inline]
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(address, buffer)
    }
}

impl<I: i2c::Write> i2c::WriteRead for I2cRef<'_, I>
where
    I: i2c::WriteRead,
{
    type Error = <I as i2c::WriteRead>::Error;

    #[inline]
    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.write_read(address, bytes, buffer)
    }
}
