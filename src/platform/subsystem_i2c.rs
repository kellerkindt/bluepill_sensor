use embedded_hal::blocking::i2c;
use lm75::{Error as Lm75Error, Lm75, SlaveAddr};
use nb::Error as NbError;
use stm32f1xx_hal::gpio::gpiob::{PB10, PB11};
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
use stm32f1xx_hal::i2c::{BlockingI2c, Error as I2cError};

pub struct I2cBus {
    pub(super) i2c: BlockingI2c<
        stm32f1xx_hal::pac::I2C2,
        (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>),
    >,
}

impl I2cBus {
    #[allow(unused)]
    pub fn init(&mut self) {
        #[cfg(feature = "board-rev-3-1")]
        let _ = self.set_expander_gpio(0xFF);
    }

    #[allow(unused)]
    pub fn read_temperature(&mut self) -> Result<f32, Lm75Error<NbError<I2cError>>> {
        Lm75::new(I2cRef(&mut self.i2c), SlaveAddr::Default).read_temperature()
    }

    #[allow(unused)]
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

    #[allow(unused)]
    #[cfg(feature = "board-rev-3-1")]
    pub fn set_expander_gpio(
        &mut self,
        status: u8,
    ) -> Result<(), pcf857x::Error<nb::Error<I2cError>>> {
        pcf857x::Pcf8574::new(I2cRef(&mut self.i2c), pcf857x::SlaveAddr::Default).set(status)
    }

    #[allow(unused)]
    #[cfg(feature = "board-rev-3-1")]
    pub fn set_expander_gpio_blocking(
        &mut self,
        status: u8,
    ) -> Result<(), pcf857x::Error<I2cError>> {
        loop {
            match self.set_expander_gpio(status) {
                Ok(v) => break Ok(v),
                Err(pcf857x::Error::I2C(nb::Error::WouldBlock)) => continue,
                Err(pcf857x::Error::I2C(nb::Error::Other(e))) => break Err(pcf857x::Error::I2C(e)),
                Err(pcf857x::Error::CouldNotAcquireDevice) => {
                    break Err(pcf857x::Error::CouldNotAcquireDevice)
                }
                Err(pcf857x::Error::InvalidInputData) => {
                    break Err(pcf857x::Error::InvalidInputData)
                }
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
