use core::convert::Infallible;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

pub trait OutputPinInfallible {
    fn set_low_infallible(&mut self);

    fn set_high_infallible(&mut self);
}

impl<O: OutputPin<Error = Infallible>> OutputPinInfallible for O {
    fn set_low_infallible(&mut self) {
        self.set_low().unwrap()
    }

    fn set_high_infallible(&mut self) {
        self.set_high().unwrap()
    }
}

pub trait InputPinInfallible {
    fn is_low_infallible(&self) -> bool;

    fn is_high_infallible(&self) -> bool;
}

impl<I: InputPin<Error = Infallible>> InputPinInfallible for I {
    fn is_low_infallible(&self) -> bool {
        self.is_low().unwrap()
    }

    fn is_high_infallible(&self) -> bool {
        self.is_high().unwrap()
    }
}
