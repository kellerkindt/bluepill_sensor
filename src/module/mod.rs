#[cfg(feature = "module-em")]
pub mod em;

#[cfg(feature = "module-em")]
pub use em::ElectricityMeter as FeaturedModule;

#[cfg(feature = "module-ps")]
pub mod ps;

#[cfg(feature = "module-ps")]
pub use ps::PumpingSystem as FeaturedModule;

#[cfg(feature = "module-env")]
pub mod env;

#[cfg(feature = "module-env")]
pub use env::EnvironmentalModule as FeaturedModule;

#[cfg(feature = "module-solax")]
pub mod solax;

#[cfg(feature = "module-solax")]
pub use solax::SolaxModbusModule as FeaturedModule;

#[cfg(not(feature = "module-specified"))]
pub mod nop;

#[cfg(not(feature = "module-specified"))]
pub use nop::NopModule as FeaturedModule;

use crate::platform::Action;
use crate::platform::HandleError;
use crate::platform::Platform;
use core::convert::Infallible;
use sensor_common::props::{ModuleId, Property};
use sensor_common::Read;
use sensor_common::Request;
use sensor_common::Write;
use stm32f1xx_hal::gpio::gpioa::*;
use stm32f1xx_hal::gpio::gpiob::*;
use stm32f1xx_hal::gpio::{Debugger, Floating, Input};
use stm32f1xx_hal::rcc::{Clocks, APB1, APB2};
use stm32f1xx_hal::{afio, flash};

pub trait RequestHandler {
    fn try_handle_request(
        &mut self,
        platform: &mut Platform,
        request: Request,
        request_payload: &mut impl sensor_common::Read,
        response_writer: &mut impl sensor_common::Write,
    ) -> Result<Action, HandleError>;
}

impl RequestHandler for Infallible {
    fn try_handle_request(
        &mut self,
        _platform: &mut Platform,
        request: Request,
        _request_payload: &mut impl Read,
        _response_writer: &mut impl Write,
    ) -> Result<Action, HandleError> {
        Ok(Action::HandleRequest(request))
    }
}

pub trait ModuleBuilder<M: Module<Builder = Self>>: Sized {
    fn build(
        platform: &mut Platform,
        constraints: &mut PlatformConstraints,
        peripherals: ModulePeripherals,
    ) -> M;
}

pub trait Module: RequestHandler + Sized + 'static {
    type Builder: ModuleBuilder<Self>;

    const PROPERTIES: &'static [Property<Platform, Self>];

    fn module_id(&self) -> ModuleId;

    fn update(&mut self, platform: &mut Platform);
}

/// TODO not stacking compatible yet (Options for the pins might be possible, but introduces runtime checks + failures, sharing buses requires (async) Mutex?)
/// Represents the left side of [bluepill board](https://raw.githubusercontent.com/stm32-rs/stm32f1xx-hal/master/BluePillPinout.jpg)
pub struct ModulePeripherals {
    pub pin_25: PB12<Input<Floating>>,
    pub pin_26: PB13<Input<Floating>>,
    pub pin_27: PB14<Input<Floating>>,
    pub pin_28: PB15<Input<Floating>>,
    pub pin_29: PA8<Input<Floating>>,
    pub pin_30: PA9<Input<Floating>>,
    pub pin_31: PA10<Input<Floating>>,
    pub pin_32: PA11<Input<Floating>>,
    pub pin_33: PA12<Input<Floating>>,
    pub pin_38: PA15<Debugger>,
    pub pin_39: PB3<Debugger>,
    pub pin_40: PB4<Debugger>,
    pub pin_41: PB5<Input<Floating>>,
    pub pin_42: PB6<Input<Floating>>,
    pub pin_43: PB7<Input<Floating>>,
    pub pin_45: PB8<Input<Floating>>,
    pub pin_46: PB9<Input<Floating>>,

    pub i2c1: stm32f1xx_hal::stm32::I2C1,

    pub usart1: stm32f1xx_hal::stm32::USART1,
    pub usart2: stm32f1xx_hal::stm32::USART2,
    pub usart3: stm32f1xx_hal::stm32::USART3,
}

pub struct PlatformConstraints {
    pub flash: flash::Parts,
    pub clocks: Clocks,
    pub afio: afio::Parts,
    pub apb1: APB1,
    pub apb2: APB2,
    pub gpioa_crl: stm32f1xx_hal::gpio::gpioa::CRL,
    pub gpioa_crh: stm32f1xx_hal::gpio::gpioa::CRH,
    pub gpiob_crl: stm32f1xx_hal::gpio::gpiob::CRL,
    pub gpiob_crh: stm32f1xx_hal::gpio::gpiob::CRH,
    pub gpioc_crl: stm32f1xx_hal::gpio::gpioc::CRL,
    pub gpioc_crh: stm32f1xx_hal::gpio::gpioc::CRH,
}

pub struct MutRef<T>(pub T);

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
