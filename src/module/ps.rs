use crate::module::{
    Module, ModuleBuilder, ModulePeripherals, MutRef, PlatformConstraints, RequestHandler,
};
use crate::platform::{Action, HandleError, Platform};
use ads1x1x::{FullScaleRange, SlaveAddr};
use arrayvec::ArrayString;
use byteorder::{ByteOrder, NetworkEndian};
use embedded_hal::adc::OneShot;
use sensor_common::props::Property;
use sensor_common::props::{ModuleId, QueryComplexity};
use sensor_common::{Bus, Format, Read, Request, Response, Type, Write};
use stm32f1xx_hal::gpio::gpiob::*;
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
use stm32f1xx_hal::i2c;
use stm32f1xx_hal::i2c::BlockingI2c;
use stm32f1xx_hal::time::U32Ext;

#[deprecated]
#[allow(unused)]
mod expander {
    pub const P7: u8 = 0b1 << 7;
    pub const P6: u8 = 0b1 << 6;
    pub const P5: u8 = 0b1 << 5;
    pub const P4: u8 = 0b1 << 4;
    pub const P3: u8 = 0b1 << 3;
    pub const P2: u8 = 0b1 << 2;
    pub const P1: u8 = 0b1 << 1;
    pub const P0: u8 = 0b1 << 0;

    // p7: low-active pump
    // p6: low-active big alarm
    // p5: low-active small alarm

    pub const PUMP: MaskedPin = MaskedPin(P7, true);
    pub const BIG_ALARM: MaskedPin = MaskedPin(P6, true);
    pub const SMALL_ALARM: MaskedPin = MaskedPin(P5, true);

    pub struct MaskedPin(u8, bool);

    impl MaskedPin {
        pub fn to_mask(&self, value: bool) -> u8 {
            if value ^ self.1 {
                self.0
            } else {
                0
            }
        }
    }
}

const ALERT_TIMEOUT: u64 = 1_000 * 60;
const ALERT_MISSING_PUMP_AFTER_MS: u64 = 1_000 * 60 * 60 * 12;
const MIN_UPTIME_BEFORE_PANIC_MS: u64 = 1_000 * 10;
const IRREGULAR_HIGH_PUMP_BURSTS_WINDOW_MS: u64 = 1_000 * 60 * 2;

pub struct PSBuilder;

impl ModuleBuilder<PumpingSystem> for PSBuilder {
    fn build(
        platform: &mut Platform,
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
            pump_state: PumpState::new(platform.system.info.uptime_ms()),
            alert_active_since_timestamp_ms: None,
            warn_since_timestamp_ms: None,
            pump_state_timestamp: OnOffTimestamp::Off(platform.system.info.uptime_ms()),
        }
    }
}

#[derive(Copy, Clone)]
enum OnOffTimestamp {
    On(u64),
    Off(u64),
}

impl OnOffTimestamp {
    pub fn on_since(&self) -> Option<u64> {
        if let OnOffTimestamp::On(time) = self {
            Some(*time)
        } else {
            None
        }
    }

    pub fn off_since(&self) -> Option<u64> {
        if let OnOffTimestamp::Off(time) = self {
            Some(*time)
        } else {
            None
        }
    }
}

pub struct PumpingSystem {
    i2c: BlockingI2c<
        stm32f1xx_hal::pac::I2C1,
        (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>),
    >,
    pump_state: PumpState,
    alert_active_since_timestamp_ms: Option<u64>,
    warn_since_timestamp_ms: Option<u64>,
    pump_state_timestamp: OnOffTimestamp,
}

impl PumpingSystem {
    pub fn load_all_adc_values(&mut self) -> Result<[f32; 4], HandleError> {
        let mut adc = ads1x1x::Ads1x1x::new_ads1115(MutRef(&mut self.i2c), SlaveAddr::new_gnd());
        adc.set_full_scale_range(FullScaleRange::Within4_096V)?;
        Ok([
            block!(adc.read(&mut ads1x1x::channel::SingleA0))? as f32 / 4096_f32,
            block!(adc.read(&mut ads1x1x::channel::SingleA1))? as f32 / 4096_f32,
            block!(adc.read(&mut ads1x1x::channel::SingleA2))? as f32 / 4096_f32,
            block!(adc.read(&mut ads1x1x::channel::SingleA3))? as f32 / 4096_f32,
        ])
    }

    pub fn update_gpio(&mut self, timestamp_ms: u64) -> Result<(), HandleError> {
        let mut pcf = pcf857x::Pcf8574a::new(
            MutRef(&mut self.i2c),
            pcf857x::SlaveAddr::Alternative(false, false, false),
        );

        let mut status = 0b0000_0000;

        let warning_active = self
            .warn_since_timestamp_ms
            .map(|t| {
                let warning_counter = timestamp_ms.saturating_sub(t) / ALERT_TIMEOUT;
                warning_counter % 2 == 0
            })
            .unwrap_or(false);

        let alert_active = self
            .alert_active_since_timestamp_ms
            .map(|t| {
                let alert_counter = timestamp_ms.saturating_sub(t) / ALERT_TIMEOUT;
                alert_counter % 2 == 0
            })
            .unwrap_or(false);

        status |= expander::SMALL_ALARM.to_mask(warning_active);
        status |= expander::BIG_ALARM.to_mask(alert_active);
        status |= expander::PUMP.to_mask(
            self.pump_state_timestamp
                .on_since()
                .map(|t| timestamp_ms.saturating_sub(t) <= PUMP_MAX_ON_TIME_MS)
                .unwrap_or(false),
        );

        pcf.set(status)?;
        Ok(())
    }

    pub fn update_warning_and_alert(&mut self, _timestamp_ms: u64) {
        if let Some(since) = self.pump_state.pre_failure_since() {
            self.warn_since_timestamp_ms = Some(since);
        }
        if let Some(since) = self.pump_state.failed_since() {
            self.raise_alert(since);
        }
    }

    pub fn raise_alert(&mut self, timestamp_ms: u64) {
        if let Some(current) = self.alert_active_since_timestamp_ms {
            if current < timestamp_ms {
                self.alert_active_since_timestamp_ms = Some(current);
            }
        } else {
            self.alert_active_since_timestamp_ms = Some(timestamp_ms);
        }
    }
}

impl Module for PumpingSystem {
    type Builder = PSBuilder;
    const PROPERTIES: &'static [Property<Platform, Self>] = &[
        Property {
            id: &[0x00, 0x00],
            type_hint: Some(Type::DynString),
            description: Some("pump-state"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    let str = module.pump_state.as_str().as_bytes();
                    let len = str.len().min(u8::MAX as usize) as u8;
                    Ok(write.write_u8(len)? + write.write_all(&str[..len as usize])?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x01],
            type_hint: Some(Type::I64),
            description: Some("pump-state-timestamp"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    let timestamp_ms = platform.system.info.uptime_ms();
                    write.write_all(
                        &match module.pump_state_timestamp {
                            OnOffTimestamp::Off(time) => timestamp_ms.saturating_sub(time) as i64,
                            OnOffTimestamp::On(time) => {
                                timestamp_ms.saturating_sub(time) as i64 * -1
                            }
                        }.to_be_bytes()
                    )
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x02],
            type_hint: Some(Type::U64),
            description: Some("alert-since-timestamp"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    write.write_all(
                        &module
                            .alert_active_since_timestamp_ms
                            .map(|t| platform.system.info.uptime_ms().saturating_sub(t))
                            .unwrap_or(0).to_be_bytes()
                    )
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x03],
            type_hint: Some(Type::U64),
            description: Some("warn-since-timestamp"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    write.write_all(
                        &module
                            .warn_since_timestamp_ms
                            .map(|t| platform.system.info.uptime_ms().saturating_sub(t))
                            .unwrap_or(0)
                            .to_be_bytes()
                    )
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x04],
            type_hint: Some(Type::U64),
            description: Some("warn-since-timestamp-burst-window"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    write.write_all(
                        &module
                            .warn_since_timestamp_ms
                            .filter(|t| {
                                platform.system.info.uptime_ms().saturating_sub(*t) <= IRREGULAR_HIGH_PUMP_BURSTS_WINDOW_MS
                            })
                            .unwrap_or(0)
                            .to_be_bytes()
                    )
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x00],
            type_hint: Some(Type::F32),
            description: Some("adc0-fill"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    write.write_all(
                        &module
                            .load_all_adc_values()
                            .map_err(|_| sensor_common::Error::UnexpectedEOF)?[0]
                            .to_be_bytes()
                    )
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x01],
            type_hint: Some(Type::F32),
            description: Some("adc1"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    write.write_all(
                        &module
                            .load_all_adc_values()
                            .map_err(|_| sensor_common::Error::UnexpectedEOF)?[1]
                            .to_be_bytes()
                    )
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x02],
            type_hint: Some(Type::F32),
            description: Some("adc2-vcc"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    write.write_all(
                        &module
                            .load_all_adc_values()
                            .map_err(|_| sensor_common::Error::UnexpectedEOF)?[2]
                            .to_be_bytes()
                    )
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x03],
            type_hint: Some(Type::F32),
            description: Some("adc3"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut PumpingSystem, write| {
                    write.write_all(
                        &module
                            .load_all_adc_values()
                            .map_err(|_| sensor_common::Error::UnexpectedEOF)?[3]
                            .to_be_bytes()
                    )
                }
            },
            write: None,
        },
    ];

    fn module_id(&self) -> ModuleId {
        ModuleId {
            group: 0,
            id: 2,
            ext: 0,
        }
    }

    fn update(&mut self, platform: &mut Platform) {
        let timestamp_ms = platform.system.info.uptime_ms();

        if let Ok([fill, _battery, _vcc5v, _pressure]) = self.load_all_adc_values() {
            self.pump_state.update(timestamp_ms, fill);
            self.update_warning_and_alert(timestamp_ms);

            if self.pump_state.should_pump() {
                if self.pump_state_timestamp.on_since().is_none() {
                    self.pump_state_timestamp = OnOffTimestamp::On(timestamp_ms);
                }
            } else if self.pump_state_timestamp.off_since().is_none() {
                self.pump_state_timestamp = OnOffTimestamp::Off(timestamp_ms);
            }
        } else {
            self.raise_alert(timestamp_ms);
        }

        if self
            .pump_state_timestamp
            .off_since()
            .map(|t| timestamp_ms.saturating_sub(t) >= ALERT_MISSING_PUMP_AFTER_MS)
            .unwrap_or(false)
        {
            self.raise_alert(timestamp_ms);
        }

        if let Err(e) = self.update_gpio(timestamp_ms) {
            if platform.system.info.uptime_ms() >= MIN_UPTIME_BEFORE_PANIC_MS {
                panic!("GPIO failed: {:?}", e);
            }
        }
    }
}

impl RequestHandler for PumpingSystem {
    fn try_handle_request(
        &mut self,
        platform: &mut Platform,
        request: Request,
        _request_payload: &mut impl Read,
        response_writer: &mut impl Write,
    ) -> Result<Action, HandleError> {
        match request {
            Request::ReadSpecified(id, Bus::Custom(c)) if c <= 3 => {
                let mut adc =
                    ads1x1x::Ads1x1x::new_ads1115(MutRef(&mut self.i2c), SlaveAddr::new_gnd());
                if let Ok(value) = adc
                    .set_full_scale_range(FullScaleRange::Within4_096V)
                    .and_then(|_| match c {
                        0 => block!(adc.read(&mut ads1x1x::channel::SingleA0)),
                        1 => block!(adc.read(&mut ads1x1x::channel::SingleA1)),
                        2 => block!(adc.read(&mut ads1x1x::channel::SingleA2)),
                        3 => block!(adc.read(&mut ads1x1x::channel::SingleA3)),
                        _ => unreachable!(),
                    })
                {
                    let mut bytes = [0u8; 4];
                    NetworkEndian::write_f32(&mut bytes, value as f32 * 0.125_f32 / 1024_f32);
                    Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                    response_writer.write_all(&bytes[..])?;
                } else {
                    Response::NotAvailable(id).write(response_writer)?;
                }
                Ok(Action::SendResponse)
            }
            /*
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
            }*/
            Request::ReadSpecified(id, Bus::Custom(200)) => {
                use core::fmt::Write;
                let mut buffer = ArrayString::<[u8; 129]>::new();
                if write!(&mut buffer, "{:?}", self.pump_state).is_ok() {
                    let bytes = buffer.as_bytes();
                    Response::Ok(id, Format::ValueOnly(Type::String(bytes.len() as u8)))
                        .write(response_writer)?;
                    response_writer.write_all(bytes)?;
                } else {
                    Response::NotAvailable(id).write(response_writer)?;
                }
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(201)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let timestamp_ms = platform.system.info.uptime_ms();
                let mut buffer = [0u8; 4];
                NetworkEndian::write_f32(
                    &mut buffer[..],
                    match self.pump_state_timestamp {
                        OnOffTimestamp::Off(time) => timestamp_ms.saturating_sub(time) as f32,
                        OnOffTimestamp::On(time) => {
                            timestamp_ms.saturating_sub(time) as f32 * -1_f32
                        }
                    },
                );
                response_writer.write_all(&buffer[..])?;
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(202)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let timestamp_ms = platform.system.info.uptime_ms();
                let mut buffer = [0u8; 4];
                NetworkEndian::write_f32(
                    &mut buffer[..],
                    self.alert_active_since_timestamp_ms
                        .map(|t| timestamp_ms.saturating_sub(t))
                        .unwrap_or(0) as f32,
                );
                response_writer.write_all(&buffer[..])?;
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(203)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let timestamp_ms = platform.system.info.uptime_ms();
                let mut buffer = [0u8; 4];
                NetworkEndian::write_f32(
                    &mut buffer[..],
                    self.warn_since_timestamp_ms
                        .map(|t| timestamp_ms.saturating_sub(t))
                        .unwrap_or(0) as f32,
                );
                response_writer.write_all(&buffer[..])?;
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(204)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let timestamp_ms = platform.system.info.uptime_ms();
                let mut buffer = [0u8; 4];
                NetworkEndian::write_f32(
                    &mut buffer[..],
                    self.warn_since_timestamp_ms
                        .filter(|t| {
                            timestamp_ms.saturating_sub(*t) <= IRREGULAR_HIGH_PUMP_BURSTS_WINDOW_MS
                        })
                        .unwrap_or(0) as f32,
                );
                response_writer.write_all(&buffer[..])?;
                Ok(Action::SendResponse)
            }
            _ => Ok(Action::HandleRequest(request)),
        }
    }
}

#[derive(Debug)]
enum PumpState {
    NoticingFill {
        /// the timestamp at which this state was initially entered
        timestamp_ms: u64,
        /// the current fill value (updated only on significant change)
        fill: f32,
        /// the timestamp since when the fill value doesn't change significantly
        level_since_ms: Option<u64>,
        pump_cycle: u8,
        prev_pre_fails: u8,
    },
    Pumping {
        timestamp_ms: u64,
        low_fill_timestamp_ms: Option<u64>,
        pump_cycle: u8,
        prev_pre_fails: u8,
    },
    Cooldown {
        timestamp_ms: u64,
        pump_cycle: u8,
        prev_pre_fails: u8,
    },
    PreFail {
        /// the timestamp at which this state was initially entered
        timestamp_ms: u64,
        /// previous pre fails
        prev_pre_fails: u8,
    },
    Failure {
        /// the timestamp at which this state was initially entered
        timestamp_ms: u64,
    },
}

const PUMP_CYCLES_BEFORE_PREFAIL: u8 = 3;
const PUMP_PREFAIL_COOLDOWN_INCREMENT_MS: u64 = 1_000 * 60 * 15;
const PUMP_PREFAILS_BEFORE_FAILURE: u8 = 10;
const PUMP_CRIT_THRESHOLD: f32 = 1.25_f32;
const PUMP_HIGH_THRESHOLD: f32 = 0.95_f32;
const PUMP_LOW_THRESHOLD: f32 = 0.2_f32;
const PUMP_SIGNIFICANT_CHANGE: f32 = 0.05_f32;
const PUMP_MIN_ON_TIME_MS: u64 = 1_750;
const PUMP_NOMINAL_ON_TIME_MS: u64 = 4_000;
const PUMP_MAX_ON_TIME_MS: u64 = 10_000;
const PUMP_COOLDOWN_MS: u64 = 2_000;
const PUMP_IF_LEVEL_FOR_AT_LEAST_MS: u64 = 1_000 * 10;
const PUMP_ANYWAY_ONCE_NOTICED_FILLING_MS: u64 = 1_000 * 60 * 5;

impl PumpState {
    pub fn new(timestamp_ms: u64) -> Self {
        PumpState::Cooldown {
            timestamp_ms,
            pump_cycle: 0,
            prev_pre_fails: 0,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            PumpState::NoticingFill { .. } => "noticing-fill",
            PumpState::Pumping { .. } => "pumping",
            PumpState::Cooldown { .. } => "cooldown",
            PumpState::PreFail { .. } => "pre-fail",
            PumpState::Failure { .. } => "failure",
        }
    }

    pub fn update(&mut self, current_timestamp_ms: u64, current_fill: f32) {
        match self {
            PumpState::NoticingFill {
                timestamp_ms,
                fill,
                level_since_ms,
                pump_cycle,
                prev_pre_fails: pre_fails,
            } => {
                if current_fill >= *fill + PUMP_SIGNIFICANT_CHANGE {
                    *fill = current_fill;
                    *level_since_ms = Some(current_timestamp_ms);
                }
                let is_level_long_enough = current_timestamp_ms
                    .saturating_sub(level_since_ms.unwrap_or(*timestamp_ms))
                    > PUMP_IF_LEVEL_FOR_AT_LEAST_MS;
                let pump_anyway = current_timestamp_ms.saturating_sub(*timestamp_ms)
                    > PUMP_ANYWAY_ONCE_NOTICED_FILLING_MS;
                let is_critical = current_fill > PUMP_CRIT_THRESHOLD;
                if is_level_long_enough || pump_anyway || is_critical {
                    *self = PumpState::Pumping {
                        timestamp_ms: current_timestamp_ms,
                        low_fill_timestamp_ms: None,
                        pump_cycle: pump_cycle.saturating_add(1),
                        prev_pre_fails: *pre_fails,
                    }
                }
            }
            PumpState::Pumping {
                timestamp_ms,
                low_fill_timestamp_ms,
                pump_cycle,
                prev_pre_fails,
            } => {
                let nominal_pump_cycle_completed =
                    current_timestamp_ms.saturating_sub(*timestamp_ms) > PUMP_NOMINAL_ON_TIME_MS;
                let early_completion = low_fill_timestamp_ms
                    .map(|t| current_timestamp_ms.saturating_sub(t) > PUMP_MIN_ON_TIME_MS)
                    .unwrap_or(false);

                if nominal_pump_cycle_completed || early_completion {
                    *self = PumpState::Cooldown {
                        timestamp_ms: current_timestamp_ms,
                        pump_cycle: *pump_cycle,
                        prev_pre_fails: *prev_pre_fails,
                    }
                } else if low_fill_timestamp_ms.is_none() && current_fill < PUMP_LOW_THRESHOLD {
                    *low_fill_timestamp_ms = Some(current_timestamp_ms);
                }
            }
            PumpState::Cooldown {
                timestamp_ms,
                pump_cycle,
                prev_pre_fails,
            } => {
                if current_timestamp_ms.saturating_sub(*timestamp_ms) > PUMP_COOLDOWN_MS {
                    if current_fill > PUMP_HIGH_THRESHOLD {
                        if *pump_cycle >= PUMP_CYCLES_BEFORE_PREFAIL {
                            if *prev_pre_fails >= PUMP_PREFAILS_BEFORE_FAILURE {
                                *self = PumpState::Failure {
                                    timestamp_ms: current_timestamp_ms,
                                }
                            } else {
                                *self = PumpState::PreFail {
                                    timestamp_ms: current_timestamp_ms,
                                    prev_pre_fails: *prev_pre_fails,
                                }
                            }
                        } else {
                            *self = PumpState::NoticingFill {
                                timestamp_ms: current_timestamp_ms,
                                fill: current_fill,
                                level_since_ms: None,
                                pump_cycle: *pump_cycle,
                                prev_pre_fails: 0,
                            }
                        }
                    } else {
                        // reset because of long pause
                        *pump_cycle = 0;
                        *prev_pre_fails = 0;
                    }
                }
            }
            PumpState::PreFail {
                timestamp_ms,
                prev_pre_fails,
            } => {
                let sleep_time = PUMP_PREFAIL_COOLDOWN_INCREMENT_MS
                    .saturating_mul(u64::from(*prev_pre_fails).saturating_add(1));
                if current_timestamp_ms.saturating_sub(*timestamp_ms) > sleep_time {
                    *self = PumpState::Pumping {
                        timestamp_ms: current_timestamp_ms,
                        low_fill_timestamp_ms: None,
                        pump_cycle: 0,
                        prev_pre_fails: prev_pre_fails.saturating_add(1),
                    }
                }
            }
            PumpState::Failure { timestamp_ms: _ } => {}
        }
    }

    pub fn should_pump(&self) -> bool {
        matches!(self, PumpState::Pumping { .. })
    }

    pub fn pre_failure_since(&self) -> Option<u64> {
        if let PumpState::PreFail { timestamp_ms, .. } = self {
            Some(*timestamp_ms)
        } else {
            None
        }
    }

    pub fn failed_since(&self) -> Option<u64> {
        if let PumpState::Failure { timestamp_ms } = self {
            Some(*timestamp_ms)
        } else {
            None
        }
    }
}

impl From<ads1x1x::Error<nb::Error<stm32f1xx_hal::i2c::Error>>> for HandleError {
    fn from(e: ads1x1x::Error<nb::Error<stm32f1xx_hal::i2c::Error>>) -> Self {
        match e {
            ads1x1x::Error::I2C(i2c) => HandleError::I2c(i2c),
            ads1x1x::Error::InvalidInputData => HandleError::Unknown, // TODO
        }
    }
}

impl From<pcf857x::Error<nb::Error<stm32f1xx_hal::i2c::Error>>> for HandleError {
    fn from(e: pcf857x::Error<nb::Error<stm32f1xx_hal::i2c::Error>>) -> Self {
        match e {
            pcf857x::Error::I2C(i2c) => HandleError::I2c(i2c),
            pcf857x::Error::InvalidInputData => HandleError::Unknown, // TODO
            pcf857x::Error::CouldNotAcquireDevice => HandleError::Unknown, // TODO
        }
    }
}
