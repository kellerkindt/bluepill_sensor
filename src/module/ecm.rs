use crate::io_utils::InputPinInfallible;
use stm32f1xx_hal::gpio::gpioa::PA12;
use stm32f1xx_hal::gpio::gpioa::PA15;
use stm32f1xx_hal::gpio::gpiob::PB3;
use stm32f1xx_hal::gpio::gpiob::PB4;
use stm32f1xx_hal::gpio::Floating;
use stm32f1xx_hal::gpio::Input;

pub struct ElectricCounterModule {
    pub pa12: PA12<Input<Floating>>,
    pub pa15: PA15<Input<Floating>>,
    pub pb3: PB3<Input<Floating>>,
    pub pb4: PB4<Input<Floating>>,

    pub ltfm1: LongTimeFreqMeasurement,
    pub ltfm2: LongTimeFreqMeasurement,
    pub ltfm3: LongTimeFreqMeasurement,
    pub ltfm4: LongTimeFreqMeasurement,
}

impl ElectricCounterModule {
    pub fn update(&mut self, time_us: u64) {
        self.ltfm1.update(time_us, self.pa12.is_high_infallible());
        self.ltfm2.update(time_us, self.pa15.is_high_infallible());
        self.ltfm3.update(time_us, self.pb3.is_high_infallible());
        self.ltfm4.update(time_us, self.pb4.is_high_infallible());
    }
}

pub struct LongTimeFreqMeasurement {
    last_value: Option<(u64, u64)>,
    current: FreqState,
}

impl LongTimeFreqMeasurement {
    pub const fn new() -> Self {
        LongTimeFreqMeasurement {
            last_value: None,
            current: FreqState::new(),
        }
    }

    pub fn update(&mut self, time_us: u64, state: bool) {
        if let Some(result_us) = self.current.update(time_us, state) {
            self.last_value = Some((time_us, result_us));
        }
    }

    pub fn value(&self, time: u64, min_age: u64) -> Option<f32> {
        if let Some((time, period_time_us)) = self.last_value {
            let period_time_us =
                period_time_us.max(self.current.time_us_since_freq_start(time).unwrap_or(0));
            if time >= min_age {
                let freq_time_s = period_time_us as f32 / 1_000_000.0_f32;
                let freq = 1.0_f32 / freq_time_s;
                return Some(freq);
            }
        }
        None
    }
}

pub enum FreqState {
    WaitingForFirstHigh,
    WaitingForFirstLow,
    WaitingForSecHigh(u64),
    WaitingForSecLow(u64),
}

impl FreqState {
    pub const fn new() -> Self {
        FreqState::WaitingForFirstHigh
    }

    pub fn time_us_since_freq_start(&self, time: u64) -> Option<u64> {
        match self {
            FreqState::WaitingForFirstHigh | FreqState::WaitingForFirstLow => None,
            FreqState::WaitingForSecHigh(start) | FreqState::WaitingForSecLow(start) => {
                time.checked_sub(*start)
            }
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new()
    }

    pub fn update(&mut self, time: u64, state: bool) -> Option<u64> {
        match self {
            FreqState::WaitingForFirstHigh if state => {
                *self = FreqState::WaitingForFirstLow;
            }
            FreqState::WaitingForFirstLow if !state => {
                *self = FreqState::WaitingForSecHigh(time);
            }
            FreqState::WaitingForSecHigh(start) if state => {
                *self = FreqState::WaitingForSecLow(*start);
            }
            FreqState::WaitingForSecLow(start) if !state => {
                let diff = time.wrapping_sub(*start);
                self.reset();
                return Some(diff);
            }
            _ => {}
        }
        None
    }
}
