use crate::module::solax::data::{LiveData, LIVE_DATA_LEN};
use crate::module::solax::msg::SolaxMessage;
use crate::module::{Module, ModuleBuilder, RequestHandler};
use crate::module::{ModulePeripherals, PlatformConstraints};
use crate::platform::{Action, DeviceInformation, HandleError, Platform};
use core::convert::Infallible;
use embedded_hal::prelude::*;
use sensor_common::props::{ModuleId, Property, QueryComplexity};
use sensor_common::{Read, Request, Type, Write};
use stm32f1xx_hal::gpio::gpioa::{PA10, PA9};
use stm32f1xx_hal::gpio::{Alternate, Floating, Input, PushPull};
use stm32f1xx_hal::pac::USART1;
use stm32f1xx_hal::serial;
use stm32f1xx_hal::serial::{Config, Serial};
use stm32f1xx_hal::time::U32Ext;

mod data;
#[allow(unused)]
mod msg;

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
                Config::default().baudrate(9600.bps()),
                constraints.clocks,
                &mut constraints.apb2,
            ),
            discovered: None,
            error: None,
        }
    }
}

pub struct SolaxModbusModule {
    usart: USART,
    discovered: Option<Discovered>,
    error: Option<&'static str>,
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
                    let mut context = Context::from(&mut module.usart, &platform.system.info);
                    let discovered = match module.discovered.as_ref().filter(|d| d.is_expired(&context.info)) {
                        Some(discovered) => &discovered,
                        None => {
                            match Discovered::try_from(&mut context) {
                                Ok(discovered) => {
                                    module.error = None;
                                    module.discovered = Some(discovered);
                                    module.discovered.as_ref().unwrap()
                                },
                                Err(e) => {
                                    module.error = Some(e.as_str());
                                    return Err(sensor_common::Error::UnexpectedEOF);
                                }
                            }
                        }
                    };

                    let mut context = Context::from(&mut module.usart, &platform.system.info);
                    let data = match discovered.query_live_data(&mut context) {
                        Ok(live_data) => live_data,
                        Err(e) => {
                            module.discovered = None;
                            module.error = Some(e.as_str());
                            return Err(sensor_common::Error::UnexpectedEOF);
                        }
                    };

                    let str = data.as_slice();
                    let len = str.len().min(u8::MAX as usize) as u8;
                    Ok(write.write_u8(len)? + write.write_all(&str[..len as usize])?)
                }
            },
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
    const TIMEOUT_MS: u64 = 500;

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
            DiscoverError::TransmissionError(TransmissionError::Timeout) => "transmission-timeout",
            DiscoverError::TransmissionError(TransmissionError::InvalidChecksum) => {
                "transmission-invalid-checksum"
            }
            DiscoverError::TransmissionError(_) => "transmission-error",
            DiscoverError::InvalidBroadcastAddress => "invalid-broadcast-address",
            DiscoverError::InvalidControlCode => "invalid-control-code",
            DiscoverError::InvalidFunctionCode => "invalid-function-code",
            DiscoverError::UnexpectedDataLength(_) => "unexpected-data-length",
            DiscoverError::InvalidAddressConfirmation => "invalid-address-confirmation",
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

        // TODO only the first byte?
        if SolaxMessage::BROADCAST_ADDRESS[0] != response.destination()[0] {
            return Err(DiscoverError::InvalidBroadcastAddress);
        }

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
            QueryError::TransmissionError(TransmissionError::Timeout) => "transmission-timeout",
            QueryError::TransmissionError(TransmissionError::InvalidChecksum) => {
                "transmission-invalid-checksum"
            }
            QueryError::TransmissionError(_) => "transmission-timeout",
            QueryError::InvalidLiveDataLength(_) => "invalid-live-data-len",
            QueryError::InvalidControlCode => "invalid-control-code",
            QueryError::InvalidFunctionCode => "invalid-function-code",
        }
    }
}

impl From<TransmissionError> for QueryError {
    fn from(e: TransmissionError) -> Self {
        Self::TransmissionError(e)
    }
}
