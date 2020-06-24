use crate::platform::DeviceInformation;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::gpio::gpioc::PC13;
use stm32f1xx_hal::gpio::{Output, PushPull};
use stm32f1xx_hal::time::MonoTimer;
use stm32f1xx_hal::watchdog::IndependentWatchdog;

pub struct System {
    pub info: DeviceInformation,

    pub delay: Delay,
    pub timer: MonoTimer,
    pub watchdog: IndependentWatchdog,

    pub led_status: PC13<Output<PushPull>>,
}
