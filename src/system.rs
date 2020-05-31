use crate::platform::DeviceInformation;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::gpio::gpioc::PC13;
use stm32f1xx_hal::gpio::{Output, PushPull};
use stm32f1xx_hal::time::MonoTimer;

pub struct System {
    pub info: DeviceInformation,

    pub delay: Delay,
    pub timer: MonoTimer,

    pub led_status: PC13<Output<PushPull>>,
}
