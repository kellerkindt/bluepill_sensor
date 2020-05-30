use crate::platform::{DeviceInformation, Platform};
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::time::MonoTimer;

pub struct System {
    pub info: DeviceInformation,

    pub delay: Delay,
    pub timer: MonoTimer,
}
