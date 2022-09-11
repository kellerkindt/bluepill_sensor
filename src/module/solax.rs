use crate::module::{Module, ModuleBuilder, RequestHandler};
use crate::module::{ModulePeripherals, PlatformConstraints};
use crate::platform::{Action, DeviceInformation, HandleError, Platform};
use core::convert::Infallible;
use embedded_hal::prelude::*;
use sensor_common::props::{ModuleId, Property, QueryComplexity};
use sensor_common::{Read, Request, Type, Write};
use solax_modbus::data::{LiveData, LIVE_DATA_LEN};
use solax_modbus::msg::SolaxMessage;
use stm32f1xx_hal::gpio::gpioa::{PA10, PA9};
use stm32f1xx_hal::gpio::{Alternate, Floating, Input, PushPull};
use stm32f1xx_hal::pac::USART1;
use stm32f1xx_hal::serial;
use stm32f1xx_hal::serial::{Config, Serial, StopBits};
use stm32f1xx_hal::time::U32Ext;

pub type USART = Serial<USART1, (PA9<Alternate<PushPull>>, PA10<Input<Floating>>)>;

pub struct SolaxModbusModuleBuilder;

impl ModuleBuilder<SolaxModbusModule> for SolaxModbusModuleBuilder {
    fn build(
        platform: &mut Platform,
        constraints: &mut PlatformConstraints,
        peripherals: ModulePeripherals,
    ) -> SolaxModbusModule {
        let _ = platform;
        SolaxModbusModule {
            usart: Serial::usart1(
                peripherals.usart1,
                (
                    peripherals
                        .pin_30
                        .into_alternate_push_pull(&mut constraints.gpioa_crh),
                    peripherals.pin_31,
                ),
                &mut constraints.afio.mapr,
                Config::default()
                    .baudrate(9600.bps())
                    .parity_none()
                    .stopbits(StopBits::STOP1),
                constraints.clocks,
                &mut constraints.apb2,
            ),
            discovered: None,
            cache: LiveDataCache::Empty,
            error: None,
        }
    }
}

pub enum LiveDataCache {
    Empty,
    Present { from_when: u64, live_data: LiveData },
}

pub struct SolaxModbusModule {
    usart: USART,
    discovered: Option<Discovered>,
    cache: LiveDataCache,
    error: Option<&'static str>,
}

impl SolaxModbusModule {
    const MAX_AGE_LIVE_DATA_MS: u64 = 750;

    pub fn retrieve_live_data(
        &mut self,
        info: &DeviceInformation,
    ) -> Result<LiveData, sensor_common::Error> {
        if let Some(live_data) = self.live_data_cached(info) {
            return Ok(live_data.clone());
        }

        let mut context = Context::from(&mut self.usart, info);
        let discovered = match self
            .discovered
            .as_ref()
            .filter(|d| d.is_expired(&context.info))
        {
            Some(discovered) => &discovered,
            None => match Discovered::try_from(&mut context) {
                Ok(discovered) => {
                    self.error = None;
                    self.discovered = Some(discovered);
                    self.discovered.as_ref().unwrap()
                }
                Err(e) => {
                    self.error = Some(e.as_str());
                    return Err(sensor_common::Error::UnexpectedEOF);
                }
            },
        };

        let mut context = Context::from(&mut self.usart, info);
        let live_data = match discovered.query_live_data(&mut context) {
            Ok(live_data) => live_data,
            Err(e) => {
                self.discovered = None;
                self.error = Some(e.as_str());
                return Err(sensor_common::Error::UnexpectedEOF);
            }
        };

        self.cache = LiveDataCache::Present {
            from_when: info.uptime_ms(),
            live_data: live_data.clone(),
        };

        Ok(live_data)
    }

    fn live_data_cached(&self, info: &DeviceInformation) -> Option<&LiveData> {
        if let LiveDataCache::Present {
            from_when,
            live_data,
        } = &self.cache
        {
            if *from_when + Self::MAX_AGE_LIVE_DATA_MS > info.uptime_ms() {
                return Some(live_data);
            }
        }
        None
    }
}

impl Module for SolaxModbusModule {
    type Builder = SolaxModbusModuleBuilder;
    const PROPERTIES: &'static [Property<Platform, Self>] = &[
        Property {
            id: &[0x00, 0x00],
            type_hint: Some(Type::DynString),
            description: Some("state"),
            complexity: QueryComplexity::low(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    let str = if module.discovered.is_some() {
                        "discovered"
                    } else if let Some(error) = module.error {
                        error
                    } else {
                        "uninitialized"
                    };
                    let str = str.as_bytes();
                    let len = str.len().min(u8::MAX as usize) as u8;
                    Ok(write.write_u8(len)? + write.write_all(&str[..len as usize])?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x00, 0x01],
            type_hint: Some(Type::DynBytes),
            description: Some("live-data-raw"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    let data = module.retrieve_live_data(&platform.system.info)?;
                    let str = data.as_slice();
                    let len = str.len().min(u8::MAX as usize) as u8;
                    Ok(write.write_u8(len)? + write.write_all(&str[..len as usize])?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x00],
            type_hint: Some(Type::U16),
            description: Some("solax-temperature"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    let bytes = module.retrieve_live_data(&platform.system.info)?.temperature();
                    Ok(write.write_all(&bytes)?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x01],
            type_hint: Some(Type::F32),
            description: Some("solax-energy-today"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .energy_today_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x02],
            type_hint: Some(Type::F32),
            description: Some("solax-pv1-voltage"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .pv1_voltage_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x03],
            type_hint: Some(Type::F32),
            description: Some("solax-pv2-voltage"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .pv2_voltage_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x04],
            type_hint: Some(Type::F32),
            description: Some("solax-pv1-current"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .pv1_current_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x05],
            type_hint: Some(Type::F32),
            description: Some("solax-pv2-current"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .pv2_current_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x06],
            type_hint: Some(Type::F32),
            description: Some("solax-ac-current"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .ac_current_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x07],
            type_hint: Some(Type::F32),
            description: Some("solax-ac-voltage"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .ac_voltage_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x08],
            type_hint: Some(Type::F32),
            description: Some("solax-ac-frequency"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .ac_frequency_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x09],
            type_hint: Some(Type::F32),
            description: Some("solax-ac-power"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .ac_power_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x0A],
            type_hint: None,
            description: Some("solax-unused"),
            complexity: QueryComplexity::high(),
            read: None,
            write: None,
        },
        Property {
            id: &[0x01, 0x0B],
            type_hint: Some(Type::F32),
            description: Some("solax-energy-total"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .energy_total_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x0C],
            type_hint: Some(Type::F32),
            description: Some("solax-runtime-total"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .runtime_total_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x0D],
            type_hint: Some(Type::U16),
            description: Some("solax-mode"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    let bytes = module.retrieve_live_data(&platform.system.info)?.mode();
                    let bytes = u16::from_be_bytes(bytes).to_be_bytes();
                    Ok(write.write_all(&bytes)?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x0E],
            type_hint: Some(Type::DynString),
            description: Some("solax-mode-str"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    let bytes = module
                        .retrieve_live_data(&platform.system.info)?
                        .mode_str_or_unknown()
                        .as_bytes();
                    Ok(write.write_u8(bytes.len() as u8)? + write.write_all(&bytes)?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x0F],
            type_hint: Some(Type::F32),
            description: Some("solax-grid-voltage-fault"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .grid_voltage_fault_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x10],
            type_hint: Some(Type::F32),
            description: Some("solax-grid-frequency-fault"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .grid_frequency_fault_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x11],
            type_hint: Some(Type::F32),
            description: Some("solax-dc-injection-fault"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .dc_injection_fault_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x12],
            type_hint: Some(Type::F32),
            description: Some("solax-temperature-fault"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .temperature_fault_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x13],
            type_hint: Some(Type::F32),
            description: Some("solax-pv1-voltage-fault"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .pv1_voltage_fault_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x14],
            type_hint: Some(Type::F32),
            description: Some("solax-pv2-voltage-fault"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .pv2_voltage_fault_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x15],
            type_hint: Some(Type::F32),
            description: Some("solax-gfc-fault"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    Ok(write.write_all(
                        &module
                            .retrieve_live_data(&platform.system.info)?
                            .gfc_fault_f32()
                            .to_be_bytes()
                    )?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x16],
            type_hint: Some(Type::U64),
            description: Some("solax-error-message"),
            complexity: QueryComplexity::high(),
            read: property_read_fn! {
                |platform, module: &mut SolaxModbusModule, write| {
                    let bytes = module.retrieve_live_data(&platform.system.info)?.error_message();
                    Ok(write.write_u8(bytes.len() as u8)? + write.write_all(&bytes)?)
                }
            },
            write: None,
        },
        Property {
            id: &[0x01, 0x17],
            type_hint: None,
            description: Some("solax-unknown"),
            complexity: QueryComplexity::high(),
            read: None,
            write: None,
        },
    ];

    fn module_id(&self) -> ModuleId {
        ModuleId {
            group: 0,
            id: 3,
            ext: 0,
        }
    }

    fn update(&mut self, platform: &mut Platform) {
        let _ = platform;
    }
}

impl RequestHandler for SolaxModbusModule {
    fn try_handle_request(
        &mut self,
        platform: &mut Platform,
        request: Request,
        request_payload: &mut impl Read,
        response_writer: &mut impl Write,
    ) -> Result<Action, HandleError> {
        let _ = platform;
        let _ = request_payload;
        let _ = response_writer;
        Ok(Action::HandleRequest(request))
    }
}

pub struct Context<'a> {
    usart: &'a mut USART,
    info: &'a DeviceInformation,
    timeout: u64,
}

impl<'a> Context<'a> {
    const TIMEOUT_MS: u64 = 850;

    pub fn from(usart: &'a mut USART, info: &'a DeviceInformation) -> Self {
        Self {
            timeout: info.uptime_ms() + Self::TIMEOUT_MS,
            usart,
            info,
        }
    }

    pub fn send_message(
        &mut self,
        message: SolaxMessage,
        payload: &[u8],
    ) -> Result<(), TransmissionError> {
        let message = message
            .with_header([0xAA, 0x55])
            .with_data_length(payload.len() as u8);

        self.write_slice(message.as_slice())?;
        self.write_slice(payload)?;

        // TODO big endian?
        self.write_slice(&message.checksum_u16(payload).to_be_bytes())
    }

    fn write_slice(&mut self, binary: &[u8]) -> Result<(), TransmissionError> {
        let timeout = self.timeout;
        let info = &self.info;
        let in_time = || timeout > info.uptime_ms();

        for byte in binary {
            if let Err(nb::Error::<Infallible>::WouldBlock) =
                block_while!(in_time(), self.usart.write(*byte))
            {
                // timeout
                return Err(TransmissionError::Timeout);
            }
        }

        Ok(())
    }

    #[inline]
    pub fn receive_message<const LEN: usize>(
        &mut self,
    ) -> Result<(SolaxMessage, [u8; LEN]), TransmissionError> {
        let message = self.receive_binary().map(SolaxMessage::from)?;
        let payload = self.receive_binary::<LEN>()?;
        let crc = self.receive_binary::<2>()?;

        if crc != message.checksum_u16(&payload).to_be_bytes() {
            return Err(TransmissionError::InvalidChecksum);
        } else {
            Ok((message, payload))
        }
    }

    pub fn receive_binary<const LEN: usize>(&mut self) -> Result<[u8; LEN], TransmissionError> {
        let timeout = self.timeout;
        let info = &self.info;
        let in_time = || timeout > info.uptime_ms();

        let mut binary = [0u8; LEN];

        for i in 0..LEN {
            match block_while!(in_time(), self.usart.read()) {
                Ok(value) => binary[i] = value,
                Err(nb::Error::WouldBlock) => return Err(TransmissionError::Timeout),
                Err(nb::Error::<serial::Error>::Other(e)) => {
                    return Err(TransmissionError::Serial(e))
                }
            }
        }

        Ok(binary)
    }
}

pub enum TransmissionError {
    Timeout,
    Serial(serial::Error),
    InvalidChecksum,
}

pub struct Discovered {
    // serial: [u8; 14],
    assigned_address: u8,
    when: u64,
}

pub enum DiscoverError {
    TransmissionError(TransmissionError),
    InvalidBroadcastAddress,
    InvalidControlCode,
    InvalidFunctionCode,
    UnexpectedDataLength(u8),
    InvalidAddressConfirmation,
}

impl DiscoverError {
    pub fn as_str(&self) -> &'static str {
        match self {
            DiscoverError::TransmissionError(TransmissionError::Timeout) => {
                "discover-transmission-timeout"
            }
            DiscoverError::TransmissionError(TransmissionError::InvalidChecksum) => {
                "discover-transmission-invalid-checksum"
            }
            DiscoverError::TransmissionError(_) => "discover-transmission-error",
            DiscoverError::InvalidBroadcastAddress => "discover-invalid-broadcast-address",
            DiscoverError::InvalidControlCode => "discover-invalid-control-code",
            DiscoverError::InvalidFunctionCode => "discover-invalid-function-code",
            DiscoverError::UnexpectedDataLength(_) => "discover-unexpected-data-length",
            DiscoverError::InvalidAddressConfirmation => "discover-invalid-address-confirmation",
        }
    }
}

impl From<TransmissionError> for DiscoverError {
    fn from(e: TransmissionError) -> Self {
        Self::TransmissionError(e)
    }
}

impl Discovered {
    pub const MAX_LIFETIME_MS: u64 = 5_000;

    pub fn try_from(ctx: &mut Context<'_>) -> Result<Discovered, DiscoverError> {
        ctx.send_message(SolaxMessage::DISCOVER_DEVICES, &[])?;

        let (response, serial_number) = ctx.receive_message::<14>()?;

        // // TODO only the first byte?
        // if SolaxMessage::BROADCAST_ADDRESS[0] != response.destination()[0] {
        //     return Err(DiscoverError::InvalidBroadcastAddress);
        // }

        if 0x10 != response.control_code() {
            return Err(DiscoverError::InvalidControlCode);
        }

        if 0x80 != response.function_code() {
            return Err(DiscoverError::InvalidFunctionCode);
        }

        if 14 != response.data_length() {
            return Err(DiscoverError::UnexpectedDataLength(response.data_length()));
        }

        let inverter_address_assignment = SolaxMessage::new()
            .with_source([0x00, 0x00])
            .with_destination([0x00, 0x00])
            .with_control_code(0x10)
            .with_function_code(0x01)
            .with_data_length(0x0F);

        let inverter_address = 0x18;

        let payload = {
            let mut payload = [0u8; 15];
            payload[..14].copy_from_slice(&serial_number);
            payload[14..].copy_from_slice(&[inverter_address]);
            payload
        };

        ctx.send_message(inverter_address_assignment, &payload)?;

        let (confirmation, data) = ctx.receive_message::<1>()?;

        if 0x10 != confirmation.control_code() {
            return Err(DiscoverError::InvalidControlCode);
        }

        if 0x81 != confirmation.function_code() {
            return Err(DiscoverError::InvalidFunctionCode);
        }

        // TODO rly? 0x06?
        if 0x06 != data[0] {
            return Err(DiscoverError::InvalidAddressConfirmation);
        }

        Ok(Discovered {
            // serial: serial_number,
            assigned_address: inverter_address,
            when: ctx.info.uptime_ms(),
        })
    }

    #[inline]
    pub fn expires_at_ms(&self) -> u64 {
        self.when + Self::MAX_LIFETIME_MS
    }

    #[inline]
    pub fn is_expired(&self, info: &DeviceInformation) -> bool {
        self.expires_at_ms() < info.uptime_ms()
    }

    #[inline]
    pub fn query_live_data(&self, ctx: &mut Context<'_>) -> Result<LiveData, QueryError> {
        let query_message = SolaxMessage::new()
            .with_source([0x01, 0x00])
            .with_destination([0x00, self.assigned_address])
            .with_control_code(0x11)
            .with_function_code(0x02)
            .with_data_length(0x00);

        ctx.send_message(query_message, &[])?;

        let (response, live_data) = ctx.receive_message()?;

        if usize::from(response.data_length()) != LIVE_DATA_LEN {
            return Err(QueryError::InvalidLiveDataLength(response.data_length()));
        }

        if 0x11 != response.control_code() {
            return Err(QueryError::InvalidControlCode);
        }

        // TODO or 0x82?
        if 0x82 != response.function_code() {
            return Err(QueryError::InvalidFunctionCode);
        }

        Ok(LiveData::from(live_data))
    }
}

pub enum QueryError {
    TransmissionError(TransmissionError),
    InvalidLiveDataLength(u8),
    InvalidControlCode,
    InvalidFunctionCode,
}

impl QueryError {
    pub fn as_str(&self) -> &'static str {
        match self {
            QueryError::TransmissionError(TransmissionError::Timeout) => {
                "query-transmission-timeout"
            }
            QueryError::TransmissionError(TransmissionError::InvalidChecksum) => {
                "query-transmission-invalid-checksum"
            }
            QueryError::TransmissionError(_) => "query-transmission-timeout",
            QueryError::InvalidLiveDataLength(_) => "query-invalid-live-data-len",
            QueryError::InvalidControlCode => "query-invalid-control-code",
            QueryError::InvalidFunctionCode => "query-invalid-function-code",
        }
    }
}

impl From<TransmissionError> for QueryError {
    fn from(e: TransmissionError) -> Self {
        Self::TransmissionError(e)
    }
}
