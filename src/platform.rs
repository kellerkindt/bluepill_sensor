use crate::cnf::NetworkConfiguration;
use crate::ds93c46::DS93C46;
use crate::io_utils::{InputPinInfallible, OutputPinInfallible, ToggleableOutputPinInfallible};
use crate::module::{Module, ModuleBuilder, ModulePeripherals, PlatformConstraints};
use crate::props::PROPERTIES;
use crate::system::System;
use byteorder::ByteOrder;
use byteorder::NetworkEndian;
use core::convert::Infallible;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::Read;
use embedded_hal::blocking::i2c::Write;
use embedded_hal::blocking::i2c::WriteRead;
use embedded_hal::spi::{Mode, Phase, Polarity};
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use embedded_nal::{UdpClientStack, UdpFullStack};
use nb::Error as NbError;
use onewire;
use onewire::Sensor as OneWireSensor;
use onewire::{ds18b20, DeviceSearch};
use onewire::{Device, OneWire};
use panic_persist::get_panic_message_bytes;
use sensor_common::props::{ComponentRoot, PropertyId, PropertyReportV1};
use sensor_common::Request;
use sensor_common::Response;
use sensor_common::Type;
use sensor_common::{Bus, Write as SensorWrite};
use sensor_common::{Format, Read as SensorRead};
use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::gpioa::*;
#[cfg(feature = "board-rev-2")]
use stm32f1xx_hal::gpio::gpiob::*;
#[cfg(any(feature = "board-rev-2", not(feature = "rtc")))]
use stm32f1xx_hal::gpio::gpioc::*;
use stm32f1xx_hal::gpio::Floating;
use stm32f1xx_hal::gpio::GpioExt;
use stm32f1xx_hal::gpio::Input;
use stm32f1xx_hal::gpio::Output;
use stm32f1xx_hal::gpio::PushPull;
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
#[cfg(all(not(feature = "board-rev-2"), feature = "i2c2"))]
use stm32f1xx_hal::i2c;
#[cfg(all(not(feature = "board-rev-2"), feature = "i2c2"))]
use stm32f1xx_hal::i2c::BlockingI2c;
use stm32f1xx_hal::i2c::Error as I2cError;
use stm32f1xx_hal::pac::SPI1;
use stm32f1xx_hal::rcc::RccExt;
use stm32f1xx_hal::spi;
use stm32f1xx_hal::spi::Spi;
use stm32f1xx_hal::spi::Spi1NoRemap;
use stm32f1xx_hal::time::Instant;
use stm32f1xx_hal::time::MonoTimer;
use stm32f1xx_hal::time::{Hertz, U32Ext};
use stm32f1xx_hal::watchdog::IndependentWatchdog;
use w5500::bus::FourWireRef;
use w5500::net::Ipv4Addr;
use w5500::udp::UdpSocket;
use w5500::*;

//mod stats;
#[cfg(feature = "board-rev-3-0")]
mod subsystem_i2c;
mod subsystem_spi;

#[cfg(feature = "sntp")]
mod sntp;

//use stats::Stats;
#[cfg(feature = "board-rev-3-0")]
use subsystem_i2c::I2cBus;
use subsystem_spi::SpiBus;

#[cfg(feature = "sntp")]
use sntp::Handle as SntpHandle;

pub const SOCKET_UDP_PORT: u16 = 51;

pub type SpiError = spi::Error;

/// Everything directly build onto the bluepill sensor board
pub struct Platform {
    pub(super) system: System,

    /// UserInput to restart the board
    /// # WARNING
    /// PB11<Output<PushPull>> prevents proper resets like through [`cortex_m::peripheral::SCB::sys_reset()`]
    #[cfg(feature = "board-rev-2")]
    pub(super) board_reset: PB11<Output<OpenDrain>>,
    #[cfg(not(feature = "board-rev-2"))]
    pub(super) board_reset: PA2<Output<OpenDrain>>,

    #[cfg(feature = "board-rev-2")]
    pub(super) led_blue_handle_udp: PA3<Output<PushPull>>,

    #[cfg(feature = "board-rev-2")]
    pub(super) led_yellow_boot_warning: PA2<Output<PushPull>>,
    #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
    pub(super) led_yellow_boot_warning: PC14<Output<PushPull>>,
    #[cfg(all(feature = "board-rev-3-0", not(feature = "i2c2")))]
    pub(super) led_yellow_boot_warning: PB11<Output<PushPull>>,

    #[cfg(feature = "board-rev-2")]
    pub(super) led_red_tmp_error: PA1<Output<PushPull>>,
    #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
    pub(super) led_red_tmp_error: PC15<Output<PushPull>>,
    #[cfg(all(feature = "board-rev-3-0", not(feature = "i2c2")))]
    pub(super) led_red_tmp_error: PB10<Output<PushPull>>,

    /// UserInput to reset the configuration to flash-default
    pub(super) factory_reset: PA0<Output<OpenDrain>>,

    #[cfg(feature = "board-rev-2")]
    pub(super) onewire: OneWire<PC15<Output<OpenDrain>>>,
    #[cfg(not(feature = "board-rev-2"))]
    pub(super) onewire: OneWire<PA3<Output<OpenDrain>>>,

    pub(super) network_udp: Option<UdpSocket>,
    pub(super) network_config: NetworkConfiguration,

    /// Error dump/panic of previous instance
    error_dump: Option<&'static [u8]>,

    /// The time the last error was raised
    last_error_at_uptime_ms: Option<u64>,

    /// Current errors at this moment
    errors: ErrorFlags,

    /// All errors that ever occurred while running, do not reset
    error_history: ErrorFlags,

    #[cfg(feature = "sntp")]
    pub(super) sntp: SntpHandle,

    #[cfg(feature = "board-rev-3-0")]
    pub(crate) subsystem_i2c: I2cBus,
    subsystem_spi: SpiBus,
}

impl Platform {
    pub fn save_network_configuration(&mut self) -> Result<(), ()> {
        match self.network_config.write(
            &mut self.subsystem_spi.ds93c46,
            &mut self.subsystem_spi.spi,
            &mut self.system.delay,
        ) {
            Err(_) => Err(()),
            Ok(_) => Ok(()),
        }
    }

    pub fn load_network_configuration(&mut self) -> Result<(), ()> {
        self.network_config
            .load(
                &mut self.subsystem_spi.ds93c46,
                &mut self.subsystem_spi.spi,
                &mut self.system.delay,
            )
            .map(drop)
            .map_err(drop)
    }

    pub fn init_network(&mut self) -> Result<(), ()> {
        #[cfg(feature = "sntp")]
        self.sntp.reset_network();
        self.network_udp = None;
        self.subsystem_spi
            .init_network(&mut self.system.delay, &self.network_config)?;

        if let Some(mut device) = self.subsystem_spi.network() {
            if let Some(mut socket) = device.socket().ok() {
                device.bind(&mut socket, SOCKET_UDP_PORT).map_err(drop)?;
                self.network_udp = Some(socket);
            }
            #[cfg(feature = "sntp")]
            if let Some(mut socket) = device.socket().ok() {
                device.bind(&mut socket, 43984).map_err(drop)?;
                self.sntp.set_socket(socket);
            }
        }

        Ok(())
    }

    pub fn update(&mut self, module: &mut impl Module) {
        // do all the internal updates
        self.system.info.update_uptime_offset();
        self.check_factory_reset_flag();

        #[cfg(feature = "sntp")]
        if let Err(flags) = self.sntp.update(&self.system.info, &mut self.subsystem_spi) {
            self.errors |= flags;
        }

        module.update(self);

        // check for external events
        if self.handle_udp_request(&mut [0u8; 2048], module).is_err() {
            if self.init_network().is_ok() {
                self.clear_error(ErrorFlags::NETWORK_INIT);
            } else {
                self.log_error(ErrorFlags::NETWORK_UDP);
            }
        } else {
            self.clear_error(ErrorFlags::NETWORK_UDP);
        }

        // display results
        #[allow(unused)]
        let uptime_ms = self.system.info.uptime_ms();

        // #[cfg(any(not(feature = "i2c2"), not(feature = "rtc")))]
        // if let Some(err) = self.last_error_at_uptime_ms {
        //     if uptime_ms.saturating_sub(err) >= 1_000 {
        //         self.led_red_tmp_error.set_low_infallible();
        //     }
        // }

        if let Some(mut device) = self.subsystem_spi.network() {
            if let Ok(mac) = device.mac() {
                if self.network_config.mac != mac {
                    self.log_error(ErrorFlags::NETWORK_CRASH);
                    let _ = self.init_network();
                }
            }
        }

        #[cfg(feature = "demo-temp-lm75-pcf857x")]
        if uptime_ms % 1_000 == 0 {
            if let Ok(temp) = self.subsystem_i2c.read_temperature_blocking() {
                let temp = ((temp.clamp(20.0_f32, 41.0_f32) - 20_f32) / 3_f32) as u8;
                let mut status = 0xFF >> 1;
                for _ in 0..temp {
                    status = status >> 1;
                }
                let _ = self.subsystem_i2c.set_expander_gpio(status);
            }
        }
    }

    pub fn log_error(&mut self, flags: ErrorFlags) {
        let uptime = self.system.info.uptime_ms();
        self.last_error_at_uptime_ms = Some(uptime);
        self.errors |= flags;
        self.error_history |= flags;
        #[cfg(any(not(feature = "i2c2"), not(feature = "rtc")))]
        self.led_red_tmp_error.set_high_infallible();
    }

    pub fn clear_error(&mut self, flags: ErrorFlags) -> bool {
        let has_resolved_some = self.errors & flags != ErrorFlags::empty();
        self.errors.remove(flags);
        has_resolved_some
    }

    pub fn log_boot_warning(&mut self) {
        #[cfg(any(not(feature = "i2c2"), not(feature = "rtc")))]
        self.led_yellow_boot_warning.set_high_infallible();
    }

    pub fn init_onewire(&mut self) -> Result<(), onewire::Error<Infallible>> {
        self.onewire.reset(&mut self.system.delay).map(drop)
    }

    pub fn handle_udp_request(
        &mut self,
        buffer: &mut [u8],
        module: &mut impl Module,
    ) -> Result<(), HandleError> {
        #[cfg(feature = "board-rev-2")]
        self.led_blue_handle_udp.set_high_infallible();
        let result = self.handle_udp_request_internal(buffer, module);
        #[cfg(feature = "board-rev-2")]
        self.led_blue_handle_udp.set_low_infallible();
        #[cfg(feature = "board-rev-2")]
        match result.is_err() {
            true => self.led_yellow_boot_warning.set_low_infallible(),
            false => self.led_yellow_boot_warning.set_high_infallible(),
        }
        result
    }

    fn handle_udp_request_internal(
        &mut self,
        buffer: &mut [u8],
        module: &mut impl Module,
    ) -> Result<(), HandleError> {
        if let Some((ip, port, size)) = self.receive_udp(buffer)? {
            let (whole_request_buffer, response_buffer) = buffer.split_at_mut(size);
            let writer = &mut &mut *response_buffer;
            let available = writer.available();

            let (response_size, action, request_id) = {
                let (request, request_length) = {
                    let reader = &mut &*whole_request_buffer;
                    let available = reader.available();
                    (Request::read(reader)?, available - reader.available())
                };

                let id = request.id();
                let (_request_header_buffer, request_content_buffer) =
                    whole_request_buffer.split_at_mut(request_length);

                let action =
                    self.try_handle_request(request, &mut &*request_content_buffer, writer, module);

                (available - writer.available(), action, id)
            };

            let reset = match action {
                Ok(Action::SendResponse) => {
                    self.send_udp(ip, port, &response_buffer[..response_size])?;
                    false
                }
                Ok(Action::SendResponseAndReset) => {
                    self.send_udp(ip, port, &response_buffer[..response_size])?;
                    true
                }
                Ok(Action::HandleRequest(request)) => {
                    Response::NotImplemented(request.id()).write(writer)?;
                    let response_size = available - writer.available();
                    self.send_udp(ip, port, &response_buffer[..response_size])?;
                    false
                }
                Err(e) => {
                    let to_skip = available - writer.available();
                    Response::NotAvailable(request_id).write(writer)?;
                    let response_size = available - writer.available();
                    self.send_udp(ip, port, &response_buffer[to_skip..response_size])?;
                    return Err(e)?;
                }
            };

            if reset {
                // increase possibility that packet is out
                self.system.delay.delay_ms(100_u16);
                let _ = self.load_network_configuration();
                self.reset();
            }
        }
        Ok(())
    }

    pub fn receive_udp(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<Option<(Ipv4Addr, u16, usize)>, SpiError> {
        if let Some(socket) = self.network_udp.as_mut() {
            self.subsystem_spi.network_receive_udp(buffer, socket)
        } else {
            Err(SpiError::ModeFault)
        }
    }

    pub fn send_udp(&mut self, ip: Ipv4Addr, port: u16, data: &[u8]) -> Result<(), SpiError> {
        if let Some(socket) = self.network_udp.as_mut() {
            self.subsystem_spi
                .network_send_udp_to(ip, port, data, socket)
        } else {
            Err(SpiError::ModeFault)
        }
    }

    pub fn try_handle_request<M: Module>(
        &mut self,
        request: Request,
        request_payload: &mut impl sensor_common::Read,
        response_writer: &mut impl sensor_common::Write,
        module: &mut M,
    ) -> Result<Action, HandleError> {
        match request {
            Request::DiscoverAll(id) | Request::DiscoverAllOnBus(id, Bus::OneWire) => {
                Response::Ok(id, Format::AddressOnly(Type::Bytes(8))).write(response_writer)?;
                for device in DeviceSearch::new()
                    .into_iter(&mut self.onewire, &mut self.system.delay)
                    .flat_map(Result::ok)
                {
                    match response_writer.write_all(&device.address) {
                        Err(sensor_common::Error::BufferToSmall) => break,
                        Err(e) => return Err(e)?,
                        Ok(_) => {}
                    }
                }
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(255)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let value = self
                    .sntp
                    .current_time_millis(&self.system.info)
                    .map(|d| (d / 1000) as f32)
                    .unwrap_or(f32::NAN)
                    .to_be_bytes();
                response_writer.write_all(&value)?;
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(254)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let value = self
                    .sntp
                    .last_offset_millis
                    .map(|d| (d / 1000) as f32)
                    .unwrap_or(f32::NAN)
                    .to_be_bytes();
                response_writer.write_all(&value)?;
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::Custom(253)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let value = self
                    .sntp
                    .last_update_millis
                    .map(|d| d as f32)
                    .unwrap_or(f32::NAN)
                    .to_be_bytes();
                response_writer.write_all(&value)?;
                Ok(Action::SendResponse)
            }
            #[cfg(feature = "board-rev-3-0")]
            Request::ReadSpecified(id, Bus::Custom(252)) => {
                Response::Ok(id, Format::ValueOnly(Type::F32)).write(response_writer)?;
                let value = self
                    .subsystem_i2c
                    .read_temperature_blocking()
                    .unwrap_or(f32::NAN)
                    .to_be_bytes();
                response_writer.write_all(&value)?;
                Ok(Action::SendResponse)
            }
            Request::ReadSpecified(id, Bus::OneWire) => {
                Response::Ok(id, Format::AddressValuePairs(Type::Bytes(8), Type::F32))
                    .write(response_writer)?;
                let ms_till_ready =
                    self.onewire_bulk_prepare_read(request_payload, response_writer)?;
                self.system.delay.delay_ms(ms_till_ready);
                self.onewire_bulk_read(request_payload, response_writer)?;
                Ok(Action::SendResponse)
            }
            Request::SetNetworkMac(id, mac) => {
                self.network_config.mac = MacAddress::from(mac);
                self.network_config.write(
                    &mut self.subsystem_spi.ds93c46,
                    &mut self.subsystem_spi.spi,
                    &mut self.system.delay,
                )?;
                Response::Ok(id, Format::Empty).write(response_writer)?;
                Ok(Action::SendResponseAndReset)
            }
            Request::SetNetworkIpSubnetGateway(id, ip, subnet, gateway) => {
                self.network_config.ip = Ipv4Addr::from(ip);
                self.network_config.subnet = Ipv4Addr::from(subnet);
                self.network_config.gateway = Ipv4Addr::from(gateway);
                self.network_config.write(
                    &mut self.subsystem_spi.ds93c46,
                    &mut self.subsystem_spi.spi,
                    &mut self.system.delay,
                )?;
                Response::Ok(id, Format::Empty).write(response_writer)?;
                Ok(Action::SendResponseAndReset)
            }
            Request::ListComponents(id) | Request::ListComponentsWithReportV1(id) => {
                Response::Ok(
                    id,
                    if matches!(request, Request::ListComponentsWithReportV1(_)) {
                        Format::ValueOnly(Type::DynListPropertyReportV1)
                    } else {
                        Format::AddressOnly(Type::PropertyId)
                    },
                )
                .write(response_writer)?;

                for property in PROPERTIES {
                    if matches!(request, Request::ListComponentsWithReportV1(_)) {
                        PropertyReportV1::from(property).write(response_writer)?;
                    } else {
                        PropertyId::from_slice(property.id).write(response_writer)?;
                    }
                }

                let module_id = module.module_id();
                for property in M::PROPERTIES {
                    let prefix_len = 4;
                    let id_len = property.id.len().min((u8::MAX - prefix_len) as usize) as u8;
                    let len = prefix_len + id_len;

                    response_writer.write_u8(len)?;
                    response_writer.write_all(&[
                        ComponentRoot::Module as u8,
                        module_id.group,
                        module_id.id,
                        module_id.ext,
                    ])?;
                    response_writer.write_all(&property.id[..id_len as usize])?;

                    if matches!(request, Request::ListComponentsWithReportV1(_)) {
                        PropertyReportV1::from(property).write_no_id(response_writer)?;
                    }
                }

                Ok(Action::SendResponse)
            }
            Request::RetrieveProperty(id, len) => {
                const PID_PATH_MAX_DEPTH: usize = 8_usize;
                let len = PID_PATH_MAX_DEPTH.min(usize::from(len));
                let buffer = {
                    let mut buffer = [0u8; PID_PATH_MAX_DEPTH];
                    for i in 0..len {
                        buffer[i as usize] = request_payload.read_u8()?;
                    }
                    buffer
                };
                let pid_path = &buffer[..len];

                match pid_path {
                    [component, module_group, module_id, module_ext, prop_id @ ..]
                        if *component == ComponentRoot::Module as u8
                            && *module_group == module.module_id().group
                            && *module_id == module.module_id().id
                            && *module_ext == module.module_id().ext =>
                    {
                        for property in M::PROPERTIES {
                            if property.id == prop_id {
                                drop(buffer);
                                if let Some(read_fn) = property.read.as_ref() {
                                    Response::Ok(
                                        id,
                                        Format::ValueOnly(
                                            property.type_hint.unwrap_or(Type::DynBytes),
                                        ),
                                    )
                                    .write(response_writer)?;
                                    read_fn(self, &mut *module, response_writer)?;
                                    return Ok(Action::SendResponse);
                                } else {
                                    break;
                                }
                            }
                        }
                    }
                    _ => {
                        for property in PROPERTIES {
                            if property.id == pid_path {
                                if let Some(read_fn) = property.read.as_ref() {
                                    Response::Ok(
                                        id,
                                        Format::ValueOnly(
                                            property.type_hint.unwrap_or(Type::DynBytes),
                                        ),
                                    )
                                    .write(response_writer)?;
                                    read_fn(self, &mut (), response_writer)?;
                                    return Ok(Action::SendResponse);
                                } else {
                                    break;
                                }
                            }
                        }
                    }
                }

                Response::NotAvailable(id).write(response_writer)?;
                Ok(Action::SendResponse)
            }
            Request::RetrieveErrorDump(id) => {
                if let Some(msg) = &self.error_dump {
                    let len_truncated = msg.len().min(u8::MAX as usize) as u8;
                    Response::Ok(id, Format::ValueOnly(Type::String(len_truncated)))
                        .write(response_writer)?;
                    let len_send = msg.len().min(response_writer.available());
                    response_writer.write_all(&msg[..len_send])?;
                } else {
                    Response::NotAvailable(id).write(response_writer)?;
                }
                Ok(Action::SendResponse)
            }
            Request::RetrieveDeviceInformation(id) => {
                let name = core::any::type_name::<M>().as_bytes();
                let mut buffer = [0u8; 4 + 8 + 6 + 1 + 1];
                Response::Ok(
                    id,
                    Format::ValueOnly(Type::Bytes((buffer.len() + name.len() + 1) as u8)),
                )
                .write(response_writer)?;
                NetworkEndian::write_u32(&mut buffer[0..], self.system.info.frequency().0);
                NetworkEndian::write_u64(&mut buffer[4..], self.system.info.uptime());

                buffer[12] = self.system.info.cpu_implementer();
                buffer[13] = self.system.info.cpu_variant();
                NetworkEndian::write_u16(&mut buffer[14..], self.system.info.cpu_partnumber());
                buffer[16] = self.system.info.cpu_revision();
                buffer[17] = crate::cnf::MAGIC_EEPROM_CRC_START;
                buffer[18] = 0x00;
                buffer[19] = name.len().min(usize::from(u8::MAX) - buffer.len()) as u8;

                response_writer.write_all(&buffer)?;
                response_writer.write_all(&name[..buffer[19] as usize])?;

                response_writer.write_u8(self.errors.bits)?;
                Ok(Action::SendResponse)
            }
            Request::RetrieveNetworkConfiguration(id) => {
                Response::Ok(id, Format::ValueOnly(Type::Bytes(6 + 3 * 4)))
                    .write(response_writer)?;
                response_writer.write_all(&self.network_config.mac.octets())?;
                response_writer.write_all(&self.network_config.ip.octets())?;
                response_writer.write_all(&self.network_config.subnet.octets())?;
                response_writer.write_all(&self.network_config.gateway.octets())?;
                Ok(Action::SendResponse)
            }
            Request::RetrieveVersionInformation(id) => {
                let version: &'static [u8] = env!("CARGO_PKG_VERSION").as_bytes();
                let len = version.len() as u8;
                Response::Ok(id, Format::ValueOnly(Type::String(len))).write(response_writer)?;
                response_writer.write_all(&version[..len as usize])?;
                Ok(Action::SendResponse)
            }
            _ => module.try_handle_request(self, request, request_payload, response_writer),
        }
    }

    /// Prepares the given `onewire::Device` to be read. Returns the
    /// time in milliseconds the device needs until ready for read.
    /// Fails if the device is unsupported.
    pub fn onewire_prepare_read(&mut self, device: &onewire::Device) -> Result<u16, ()> {
        match device.family_code() {
            ds18b20::FAMILY_CODE => {
                if let Ok(dev) = ds18b20::DS18B20::new(device.clone()) {
                    if let Ok(time) =
                        dev.start_measurement(&mut self.onewire, &mut self.system.delay)
                    {
                        return Ok(time);
                    }
                }
            }
            _ => {}
        }
        Err(())
    }

    pub fn onewire_bulk_prepare_read(
        &mut self,
        reader: &mut impl sensor_common::Read,
        writer: &mut impl sensor_common::Write,
    ) -> Result<u16, HandleError> {
        let mut ms_to_sleep = 0_u16;
        while reader.available() >= onewire::ADDRESS_BYTES as usize
            && writer.available() >= onewire::ADDRESS_BYTES as usize * core::mem::size_of::<f32>()
        {
            let device = Device {
                address: [
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                ],
            };

            ms_to_sleep = self
                .onewire_prepare_read(&device)
                .unwrap_or(0)
                .max(ms_to_sleep);
        }
        Ok(ms_to_sleep)
    }

    /// Reads from the given `onewire::Device`. Returns the value as f32.
    /// Fails if the device is unsupported.
    pub fn onewire_read(
        &mut self,
        device: &onewire::Device,
        retry_count_on_crc_error: u8,
    ) -> Result<f32, ()> {
        match device.family_code() {
            ds18b20::FAMILY_CODE => {
                if let Ok(dev) = ds18b20::DS18B20::new(device.clone()) {
                    for _ in 0..retry_count_on_crc_error {
                        match dev.read_temperature(&mut self.onewire, &mut self.system.delay) {
                            Ok(value) => return Ok(value as f32),
                            Err(onewire::Error::CrcMismatch(_, _)) => continue,
                            _ => break,
                        }
                    }
                }
            }
            _ => {}
        }
        Err(())
    }

    pub fn onewire_bulk_read(
        &mut self,
        reader: &mut impl sensor_common::Read,
        writer: &mut impl sensor_common::Write,
    ) -> Result<(), HandleError> {
        while reader.available() >= onewire::ADDRESS_BYTES as usize
            && writer.available() >= onewire::ADDRESS_BYTES as usize * core::mem::size_of::<f32>()
        {
            let device = Device {
                address: [
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                    reader.read_u8()?,
                ],
            };

            let value = self.onewire_read(&device, 1).unwrap_or(::core::f32::NAN);

            let mut buffer = [0u8; 4];
            NetworkEndian::write_f32(&mut buffer[..], value);

            writer.write_all(&device.address)?;
            writer.write_all(&buffer)?;
        }
        Ok(())
    }

    pub fn check_factory_reset_flag(&mut self) {
        let probe_start = self.system.timer.now();
        while self.factory_reset.is_high_infallible() {
            // pressed for longer than 3s?
            if (probe_start.elapsed() / self.system.timer.frequency().0) > 3 {
                self.network_config = NetworkConfiguration::default();
                let _ = self.save_network_configuration();
                while self.factory_reset.is_high_infallible() {
                    // turn on all debug LEDs and hold until no longer pressed
                    #[cfg(feature = "board-rev-2")]
                    self.led_blue_handle_udp.set_high_infallible();
                    #[cfg(any(not(feature = "i2c2"), not(feature = "rtc")))]
                    self.led_yellow_boot_warning.set_high_infallible();
                    #[cfg(any(not(feature = "i2c2"), not(feature = "rtc")))]
                    self.led_red_tmp_error.set_high_infallible();
                }
                self.reset();
            }
        }
    }

    pub fn reset(&mut self) {
        loop {
            self.board_reset.set_low_infallible();
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

pub struct DeviceInformation {
    frequency: Hertz,
    init: Instant,
    init_last: u32,
    uptime_offset: u64,
    cpuid: u32,
}

impl DeviceInformation {
    pub fn new(timer: &MonoTimer, cpuid: u32) -> DeviceInformation {
        DeviceInformation {
            frequency: timer.frequency(),
            init: timer.now(),
            init_last: 0,
            uptime_offset: 0,
            cpuid,
        }
    }

    pub fn frequency(&self) -> Hertz {
        self.frequency
    }

    pub fn update_uptime_offset(&mut self) {
        let elapsed = self.init.elapsed();
        let additional_offset = elapsed.wrapping_sub(self.init_last);
        self.init_last = elapsed;
        self.uptime_offset += additional_offset as u64;
    }

    pub fn uptime(&self) -> u64 {
        self.uptime_offset + self.init.elapsed().wrapping_sub(self.init_last) as u64
    }

    #[allow(unused)]
    pub fn uptime_ms(&self) -> u64 {
        self.uptime() / (self.frequency.0 / 1_000) as u64
    }

    #[allow(unused)]
    pub fn uptime_us(&self) -> u64 {
        self.uptime() / (self.frequency.0 / 1_000_000) as u64
    }

    pub fn cpu_id(&self) -> u32 {
        self.cpuid
    }

    pub fn cpu_implementer(&self) -> u8 {
        (self.cpu_id() >> 24) as u8
    }

    pub fn cpu_variant(&self) -> u8 {
        (self.cpu_id() >> 20) as u8 & 0x0F
    }

    pub fn cpu_partnumber(&self) -> u16 {
        (self.cpu_id() >> 4) as u16 & 0x0F_FF
    }

    pub fn cpu_revision(&self) -> u8 {
        self.cpu_id() as u8 & 0x0F
    }
}

pub trait FullI2C: WriteRead + Write + Read {}

impl<T> FullI2C for T where
    T: WriteRead<Error = NbError<I2cError>>
        + Write<Error = NbError<I2cError>>
        + Read<Error = NbError<I2cError>>
{
}

pub enum Action {
    SendResponse,
    SendResponseAndReset,
    HandleRequest(Request),
}

pub fn unwrap_builder() -> PlatformBuilder {
    PlatformBuilder::take_peripherals().unwrap()
}

pub struct PlatformBuilder {
    platform: Platform,
    constraints: PlatformConstraints,
    module_peripherals: ModulePeripherals,
}

impl PlatformBuilder {
    /// Tries to take all required peripherals to build the [`Platform`] and potential [`Module`]s
    /// Requires that both [`cortex_m::Peripherals`] and [`stm32f1xx_hal::pac::Peripherals`] are
    /// available and will fail if either one is unavailable through [`cp::take()`] or [`p::take()`]
    ///
    /// [`Module`]: crate::module::Module
    /// [`cp::take()`]: cortex_m::Peripherals::take()
    /// [`p::take()`]: stm32f1xx_hal::pac::Peripherals::take()
    pub fn take_peripherals() -> Option<Self> {
        let cp = cortex_m::Peripherals::take()?;
        let p = stm32f1xx_hal::pac::Peripherals::take()?;
        Some(PlatformBuilder::from((cp, p, get_panic_message_bytes())))
    }

    pub fn split(self) -> (Platform, PlatformConstraints, ModulePeripherals) {
        (self.platform, self.constraints, self.module_peripherals)
    }

    pub fn run_with_module<T: Module>(self) -> ! {
        let (mut platform, mut constraints, module) = self.split();

        #[cfg(feature = "board-rev-3-0")]
        let _ = platform.subsystem_i2c.init();
        let _ = platform.init_onewire(); // allowed to fail

        // TODO error handling
        if platform.load_network_configuration().is_err() {
            platform.log_error(ErrorFlags::NETWORK_LOAD);
            platform.log_boot_warning();
            for _ in 0..4 {
                platform.system.delay.delay_ms(1000_u16);
                platform.system.led_status.toggle_infallible();
            }
        }

        if platform.init_network().is_err() {
            platform.log_error(ErrorFlags::NETWORK_INIT);
            platform.log_boot_warning();
        }

        let mut module = T::Builder::build(&mut platform, &mut constraints, module);

        platform.system.watchdog.start(5_000.ms());

        loop {
            for _ in 0..100 {
                platform.update(&mut module);
                platform.system.watchdog.feed();
            }
            platform.system.led_status.toggle_infallible();
        }
    }
}

impl
    From<(
        cortex_m::Peripherals,
        stm32f1xx_hal::pac::Peripherals,
        Option<&'static [u8]>,
    )> for PlatformBuilder
{
    fn from(
        (mut cp, p, error_dump): (
            cortex_m::Peripherals,
            stm32f1xx_hal::pac::Peripherals,
            Option<&'static [u8]>,
        ),
    ) -> Self {
        let mut flash = p.FLASH.constrain();
        let mut rcc = p.RCC.constrain();
        let mut afio = p.AFIO.constrain(&mut rcc.apb2);

        let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut delay = stm32f1xx_hal::delay::Delay::new(cp.SYST, clocks);

        // SPI1
        let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
        let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
        let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

        let mut spi: Spi<
            SPI1,
            Spi1NoRemap,
            (
                PA5<Alternate<PushPull>>,
                PA6<Input<Floating>>,
                PA7<Alternate<PushPull>>,
            ),
            u8,
        > = Spi::spi1(
            p.SPI1,
            (sclk, miso, mosi),
            &mut afio.mapr,
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            1.mhz(), // upt to 8mhz for w5500 module, 2mhz is max for eeprom in 3.3V
            clocks,
            &mut rcc.apb2,
        );

        let mut w5500_reset = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        let mut w5500_cs = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
        let w5500 = {
            w5500_cs.set_low_infallible(); // deselect
            w5500_reset.set_high_infallible(); // request reset
            delay.delay_ms(250_u16); // pad some time for reset
            w5500_reset.set_low_infallible();
            delay.delay_ms(250_u16); // allow boot

            UninitializedDevice::new(FourWireRef::new(&mut spi, &mut w5500_cs))
                .initialize_manual(
                    MacAddress::new(0x02, 0x00, 0x00, 0x00, 0x00, 0x00),
                    NetworkConfiguration::DEFAULT_IP,
                    Default::default(),
                )
                .ok()
                .map(|device| device.deactivate().1)
        };

        let platform = Platform {
            #[cfg(feature = "board-rev-2")]
            board_reset: {
                let mut self_reset = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
                self_reset.set_high_infallible();
                self_reset
            },
            #[cfg(not(feature = "board-rev-2"))]
            board_reset: {
                let mut self_reset = gpioa.pa2.into_open_drain_output(&mut gpioa.crl);
                self_reset.set_high_infallible();
                self_reset
            },

            subsystem_spi: SpiBus {
                #[cfg(feature = "board-rev-2")]
                w5500_intr: gpiob.pb10.into_floating_input(&mut gpiob.crh),
                #[cfg(not(feature = "board-rev-2"))]
                w5500_intr: gpioa.pa1.into_floating_input(&mut gpioa.crl),
                w5500,
                w5500_cs,

                spi,

                ds93c46: {
                    let mut ds93c46_cs = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
                    ds93c46_cs.set_low_infallible(); // deselect
                    DS93C46::new(ds93c46_cs)
                },

                w5500_reset,
            },

            #[cfg(feature = "board-rev-3-0")]
            subsystem_i2c: I2cBus {
                i2c: {
                    BlockingI2c::i2c2(
                        p.I2C2,
                        (
                            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh),
                            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh),
                        ),
                        i2c::Mode::standard(100.khz()),
                        clocks,
                        &mut rcc.apb1,
                        1_000,
                        2,
                        10_000,
                        1_000_000,
                    )
                },
            },

            #[cfg(feature = "board-rev-2")]
            led_red_tmp_error: gpioa.pa1.into_push_pull_output(&mut gpioa.crl),
            #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
            led_red_tmp_error: gpioc.pc15.into_push_pull_output(&mut gpioc.crh),
            #[cfg(all(feature = "board-rev-3-0", not(feature = "i2c2")))]
            led_red_tmp_error: gpiob.pb10.into_push_pull_output(&mut gpiob.crh),

            #[cfg(feature = "board-rev-2")]
            led_yellow_boot_warning: gpioa.pa2.into_push_pull_output(&mut gpioa.crl),
            #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
            led_yellow_boot_warning: gpioc.pc14.into_push_pull_output(&mut gpioc.crh),
            #[cfg(all(feature = "board-rev-3-0", not(feature = "i2c2")))]
            led_yellow_boot_warning: gpiob.pb11.into_push_pull_output(&mut gpiob.crh),

            #[cfg(feature = "board-rev-2")]
            led_blue_handle_udp: gpioa.pa3.into_push_pull_output(&mut gpioa.crl),

            factory_reset: gpioa.pa0.into_open_drain_output(&mut gpioa.crl),

            #[cfg(feature = "board-rev-2")]
            onewire: OneWire::new(gpioc.pc15.into_open_drain_output(&mut gpioc.crh), false),
            #[cfg(not(feature = "board-rev-2"))]
            onewire: OneWire::new(gpioa.pa3.into_open_drain_output(&mut gpioa.crl), false),

            network_config: NetworkConfiguration::default(),
            network_udp: None,

            error_dump,
            last_error_at_uptime_ms: None,
            errors: ErrorFlags::empty(),
            error_history: ErrorFlags::empty(),

            system: {
                let timer = {
                    cp.DCB.enable_trace();
                    ::stm32f1xx_hal::time::MonoTimer::new(cp.DWT, cp.DCB, clocks)
                };

                System {
                    info: DeviceInformation::new(&timer, cp.CPUID.base.read()),
                    watchdog: IndependentWatchdog::new(p.IWDG),
                    delay,
                    timer,
                    led_status: gpioc.pc13.into_push_pull_output(&mut gpioc.crh),
                }
            },

            #[cfg(feature = "sntp")]
            sntp: SntpHandle::new(),
        };

        // let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        let module_peripherals = ModulePeripherals {
            pin_25: gpiob.pb12,
            pin_26: gpiob.pb13,
            pin_27: gpiob.pb14,
            pin_28: gpiob.pb15,
            pin_29: gpioa.pa8,
            pin_30: gpioa.pa9,
            pin_31: gpioa.pa10,
            pin_32: gpioa.pa11,
            pin_33: gpioa.pa12,
            pin_38: gpioa.pa15,
            pin_39: gpiob.pb3,
            pin_40: gpiob.pb4,
            pin_41: gpiob.pb5,
            pin_42: gpiob.pb6,
            pin_43: gpiob.pb7,
            pin_45: gpiob.pb8,
            pin_46: gpiob.pb9,
            i2c1: p.I2C1,
            usart1: p.USART1,
            usart2: p.USART2,
            usart3: p.USART3,
        };

        let constraints = PlatformConstraints {
            afio,
            apb1: rcc.apb1,
            apb2: rcc.apb2,
            gpioa_crl: gpioa.crl,
            gpioa_crh: gpioa.crh,
            gpiob_crl: gpiob.crl,
            gpiob_crh: gpiob.crh,
            gpioc_crl: gpioc.crl,
            gpioc_crh: gpioc.crh,
            flash,
            clocks,
        };

        PlatformBuilder {
            platform,
            constraints,
            module_peripherals,
        }
    }
}

#[derive(Debug)]
pub enum HandleError {
    Unknown,
    Spi(spi::Error),
    Parsing(sensor_common::Error),
    OneWire(onewire::Error<Infallible>),
    NotMagicCrcAtStart,
    CrcError,
    I2c(NbError<I2cError>),
}

impl From<spi::Error> for HandleError {
    fn from(e: spi::Error) -> Self {
        HandleError::Spi(e)
    }
}

impl From<sensor_common::Error> for HandleError {
    fn from(e: sensor_common::Error) -> Self {
        HandleError::Parsing(e)
    }
}

impl From<onewire::Error<Infallible>> for HandleError {
    fn from(e: onewire::Error<Infallible>) -> Self {
        HandleError::OneWire(e)
    }
}

impl From<()> for HandleError {
    fn from(_: ()) -> Self {
        HandleError::Unknown
    }
}

impl From<NbError<I2cError>> for HandleError {
    fn from(e: NbError<I2cError>) -> Self {
        HandleError::I2c(e)
    }
}

bitflags::bitflags! {
    pub struct ErrorFlags: u8 {
        const NETWORK_LOAD = 0b0000_0001;
        const NETWORK_INIT = 0b0000_0010;
        const NETWORK_UDP = 0b0000_0100;
        const NETWORK_CRASH = 0b0000_1000;
        const SNTP_REQUEST = 0b0001_0000;
        const SNTP_RESPONSE = 0b0010_0000;
    }
}
