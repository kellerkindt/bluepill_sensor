use crate::module::{
    Module, ModuleBuilder, ModulePeripherals, PlatformConstraints, RequestHandler,
};
use crate::platform::{Action, HandleError, Platform};
use ads1x1x::SlaveAddr;
use byteorder::{ByteOrder, NetworkEndian};
use embedded_hal::adc::OneShot;
use sensor_common::{Bus, Format, Read, Request, Response, Type, Write};
use stm32f1xx_hal::gpio::gpiob::*;
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
use stm32f1xx_hal::i2c;
use stm32f1xx_hal::i2c::BlockingI2c;
use stm32f1xx_hal::time::U32Ext;

pub struct PSBuilder;

impl ModuleBuilder<PumpingSystem> for PSBuilder {
    fn build(
        _platform: &mut Platform,
        constraints: &mut PlatformConstraints,
        peripherals: ModulePeripherals,
    ) -> PumpingSystem {
        PumpingSystem {
            i2c: BlockingI2c::i2c1(
                peripherals.i2c1,
                (
                    peripherals
                        .pin_45
                        .into_alternate_open_drain(&mut constraints.gpiob_crh),
                    peripherals
                        .pin_46
                        .into_alternate_open_drain(&mut constraints.gpiob_crh),
                ),
                &mut constraints.afio.mapr,
                i2c::Mode::standard(100.khz()),
                constraints.clocks,
                &mut constraints.apb1,
                1_000,
                2,
                10_000,
                1_000_000,
            ),
            state: false,
        }
    }
}

pub struct PumpingSystem {
    i2c: BlockingI2c<
        stm32f1xx_hal::pac::I2C1,
        (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>),
    >,
    state: bool,
}

impl Module for PumpingSystem {
    type Builder = PSBuilder;

    fn update(&mut self, _platform: &mut Platform) {}
}

impl RequestHandler for PumpingSystem {
    fn try_handle_request(
        &mut self,
        _platform: &mut Platform,
        request: Request,
        _request_payload: &mut impl Read,
        response_writer: &mut impl Write,
    ) -> Result<Action, HandleError> {
        match request {
            Request::ReadSpecified(id, Bus::Custom(c)) if c <= 3 => {
                let mut adc =
                    ads1x1x::Ads1x1x::new_ads1115(MutRef(&mut self.i2c), SlaveAddr::new_gnd());
                if let Ok(value) = match c {
                    0 => block!(adc.read(&mut ads1x1x::channel::SingleA0)),
                    1 => block!(adc.read(&mut ads1x1x::channel::SingleA1)),
                    2 => block!(adc.read(&mut ads1x1x::channel::SingleA2)),
                    3 => block!(adc.read(&mut ads1x1x::channel::SingleA3)),
                    _ => unreachable!(),
                } {
                    let mut bytes = [0u8; 4];
                    NetworkEndian::write_f32(&mut bytes, value as f32);
                    Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                    response_writer.write_all(&bytes[..])?;
                } else {
                    Response::NotAvailable(id).write(response_writer)?;
                }
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(x)) if x >= 100 && x <= 108 => {
                let mut pcf = pcf857x::Pcf8574a::new(
                    MutRef(&mut self.i2c),
                    pcf857x::SlaveAddr::Alternative(false, false, false),
                );
                let result = if self.state {
                    pcf.set(0x00)
                } else {
                    pcf.set(0xFF)
                };
                self.state = !self.state;
                if result.is_ok() {
                    Response::Ok(id, Format::Empty).write(response_writer)?;
                } else {
                    Response::NotAvailable(id).write(response_writer)?;
                }
                Ok(Action::SendResponse)
            }
            _ => Ok(Action::HandleRequest(request)),
        }
    }
}

pub struct MutRef<T>(T);

impl<R: embedded_hal::blocking::i2c::Read> embedded_hal::blocking::i2c::Read for MutRef<&mut R> {
    type Error = R::Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        R::read(self.0, address, buffer)
    }
}

impl<W: embedded_hal::blocking::i2c::Write> embedded_hal::blocking::i2c::Write for MutRef<&mut W> {
    type Error = W::Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        W::write(self.0, addr, bytes)
    }
}

impl<WR: embedded_hal::blocking::i2c::WriteRead> embedded_hal::blocking::i2c::WriteRead
    for MutRef<&mut WR>
{
    type Error = WR::Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        WR::write_read(self.0, address, bytes, buffer)
    }
}
