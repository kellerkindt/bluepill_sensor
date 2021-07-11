use crate::io_utils::InputPinInfallible;
use crate::module::{
    Module, ModuleBuilder, ModulePeripherals, PlatformConstraints, RequestHandler,
};
use crate::platform::HandleError;
use crate::platform::{Action, Platform};
use byteorder::ByteOrder;
use byteorder::NetworkEndian;
use sensor_common::props::Property;
use sensor_common::props::{ModuleId, QueryComplexity};
use sensor_common::Bus;
use sensor_common::Format;
use sensor_common::Response;
use sensor_common::Type;
use sensor_common::{Read, Request, Write};
use stm32f1xx_hal::gpio::gpioa::PA12;
use stm32f1xx_hal::gpio::gpioa::PA15;
use stm32f1xx_hal::gpio::gpioa::PA8;
use stm32f1xx_hal::gpio::gpiob::PB3;
use stm32f1xx_hal::gpio::gpiob::PB4;
use stm32f1xx_hal::gpio::Input;
use stm32f1xx_hal::gpio::{Floating, PullUp};

pub struct ECMBuilder;

impl ModuleBuilder<ElectricityMeter> for ECMBuilder {
    fn build(
        _platform: &mut Platform,
        constraints: &mut PlatformConstraints,
        peripherals: ModulePeripherals,
    ) -> ElectricityMeter {
        let (pa15, pb3, pb4) = constraints.afio.mapr.disable_jtag(
            peripherals.pin_38,
            peripherals.pin_39,
            peripherals.pin_40,
        );

        ElectricityMeter {
            pa8: peripherals
                .pin_29
                .into_pull_up_input(&mut constraints.gpioa_crh),
            pa12: peripherals
                .pin_33
                .into_floating_input(&mut constraints.gpioa_crh),
            pa15: pa15.into_floating_input(&mut constraints.gpioa_crh),
            pb3: pb3.into_floating_input(&mut constraints.gpiob_crl),
            pb4: pb4.into_floating_input(&mut constraints.gpiob_crl),

            garage_open_since: None,
            ltfm1: LongTimeFreqMeasurement::new(),
            ltfm2: LongTimeFreqMeasurement::new(),
            ltfm3: LongTimeFreqMeasurement::new(),
            ltfm4: LongTimeFreqMeasurement::new(),
        }
    }
}

pub struct ElectricityMeter {
    pub pa8: PA8<Input<PullUp>>,
    pub pa12: PA12<Input<Floating>>,
    pub pa15: PA15<Input<Floating>>,
    pub pb3: PB3<Input<Floating>>,
    pub pb4: PB4<Input<Floating>>,

    pub garage_open_since: Option<u64>,
    pub ltfm1: LongTimeFreqMeasurement,
    pub ltfm2: LongTimeFreqMeasurement,
    pub ltfm3: LongTimeFreqMeasurement,
    pub ltfm4: LongTimeFreqMeasurement,
}

impl Module for ElectricityMeter {
    type Builder = ECMBuilder;
    const PROPERTIES: &'static [Property<Platform, Self>] = &[
        Property {
            id: &[0x00, 0x00],
            type_hint: Some(Type::U64),
            description: Some("garage-open-since"),
            complexity: QueryComplexity::Low {
                estimated_millis: None,
            },
            read: property_read_fn! {
                |platform, module: &mut ElectricityMeter, write| {
                    let time = platform.system.info.uptime_us();
                    write.write_all(&module.garage_open_since.map(|open_since| {
                        let time_us_diff = time.saturating_sub(open_since);
                        time_us_diff as f32 / 1_000_000_f32
                    }).unwrap_or_default().to_be_bytes())
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x01],
            type_hint: Some(Type::F32),
            description: Some("ltfm1"),
            complexity: QueryComplexity::Low {
                estimated_millis: None,
            },
            read: property_read_fn! {
                |platform, module: &mut ElectricityMeter, write| {
                    let time = platform.system.info.uptime_us();
                    let min_age = time.saturating_sub(300_000_000); // 5min;
                    write.write_all(&module.ltfm1.value(time, min_age).unwrap_or_default().to_be_bytes())
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x02],
            type_hint: Some(Type::F32),
            description: Some("ltfm2"),
            complexity: QueryComplexity::Low {
                estimated_millis: None,
            },
            read: property_read_fn! {
                |platform, module: &mut ElectricityMeter, write| {
                    let time = platform.system.info.uptime_us();
                    let min_age = time.saturating_sub(300_000_000); // 5min;
                    write.write_all(&module.ltfm2.value(time, min_age).unwrap_or_default().to_be_bytes())
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x03],
            type_hint: Some(Type::F32),
            description: Some("ltfm3"),
            complexity: QueryComplexity::Low {
                estimated_millis: None,
            },
            read: property_read_fn! {
                |platform, module: &mut ElectricityMeter, write| {
                    let time = platform.system.info.uptime_us();
                    let min_age = time.saturating_sub(300_000_000); // 5min;
                    write.write_all(&module.ltfm3.value(time, min_age).unwrap_or_default().to_be_bytes())
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x04],
            type_hint: Some(Type::F32),
            description: Some("ltfm4"),
            complexity: QueryComplexity::Low {
                estimated_millis: None,
            },
            read: property_read_fn! {
                |platform, module: &mut ElectricityMeter, write| {
                    let time = platform.system.info.uptime_us();
                    let min_age = time.saturating_sub(300_000_000); // 5min;
                    write.write_all(&module.ltfm4.value(time, min_age).unwrap_or_default().to_be_bytes())
                }
            },
            write: None,
        },
    ];

    fn module_id(&self) -> ModuleId {
        ModuleId {
            group: 0,
            id: 1,
            ext: 0,
        }
    }

    fn update(&mut self, platform: &mut Platform) {
        let time_us = platform.system.info.uptime_us();
        self.ltfm1.update(time_us, self.pa12.is_high_infallible());
        self.ltfm2.update(time_us, self.pa15.is_high_infallible());
        self.ltfm3.update(time_us, self.pb3.is_high_infallible());
        self.ltfm4.update(time_us, self.pb4.is_high_infallible());

        if self.pa8.is_high_infallible() {
            if self.garage_open_since.is_none() {
                self.garage_open_since = Some(time_us);
            }
        } else {
            self.garage_open_since = None;
        }
    }
}

impl RequestHandler for ElectricityMeter {
    fn try_handle_request(
        &mut self,
        platform: &mut Platform,
        request: Request,
        _request_payload: &mut impl Read,
        response_writer: &mut impl Write,
    ) -> Result<Action, HandleError> {
        match request {
            Request::ReadSpecified(id, Bus::Custom(cid)) if cid >= 250 && cid <= 254 => {
                let time = platform.system.info.uptime_us();
                let min_age = time.saturating_sub(300_000_000); // 5min;
                let result = match cid {
                    250 => self.garage_open_since.map(|open_since| {
                        let time_us_diff = time.saturating_sub(open_since);
                        time_us_diff as f32 / 1_000_000_f32
                    }),
                    251 => self.ltfm1.value(time, min_age),
                    252 => self.ltfm2.value(time, min_age),
                    253 => self.ltfm3.value(time, min_age),
                    254 => self.ltfm4.value(time, min_age),
                    _ => unreachable!(),
                }
                .unwrap_or(0f32);

                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let mut buffer = [0u8; 4];
                NetworkEndian::write_f32(&mut buffer[..], result);
                response_writer.write_all(&buffer[..])?;
                Ok(Action::SendResponse)
            }
            _ => Ok(Action::HandleRequest(request)),
        }
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

    pub fn value(&self, system_time: u64, min_age: u64) -> Option<f32> {
        if let Some((time, period_time_us)) = self.last_value {
            let period_time_us = period_time_us.max(
                self.current
                    .time_us_since_freq_start(system_time)
                    .unwrap_or(0),
            );
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
