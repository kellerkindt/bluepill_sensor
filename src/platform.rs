use crate::cnf::NetworkConfiguration;
use crate::ds93c46::DS93C46;
use crate::io_utils::{InputPinInfallible, OutputPinInfallible, ToggleableOutputPinInfallible};
use crate::module::{Module, ModuleBuilder, ModulePeripherals, PlatformConstraints};
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
use nb::Error as NbError;
use onewire;
use onewire::Sensor as OneWireSensor;
use onewire::{ds18b20, DeviceSearch};
use onewire::{Device, OneWire};
use panic_persist::get_panic_message_bytes;
use sensor_common::Request;
use sensor_common::Response;
use sensor_common::Type;
use sensor_common::{Bus, Write as SensorWrite};
use sensor_common::{Format, Read as SensorRead};
use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::gpioa::*;
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
use w5500::*;

pub const SOCKET_UDP: Socket = Socket::Socket0;
pub const SOCKET_UDP_PORT: u16 = 51;

pub type SpiError = spi::Error;
pub type W5500CsError = Infallible;

/// Everything directly build onto the bluepill sensor board
pub struct Platform {
    pub(super) system: System,
    pub(super) watchdog: IndependentWatchdog,

    /// UserInput to restart the board
    /// # WARNING
    /// PB11<Output<PushPull>> prevents proper resets like through [`cortex_m::peripheral::SCB::sys_reset()`]
    #[cfg(feature = "board-rev-2")]
    pub(super) board_reset: PB11<Output<OpenDrain>>,
    #[cfg(not(feature = "board-rev-2"))]
    pub(super) board_reset: PA2<Output<OpenDrain>>,

    /// W5500 interrupt pin
    #[allow(unused)]
    #[cfg(feature = "board-rev-2")]
    pub(super) w5500_intr: PB10<Input<Floating>>,
    #[allow(unused)]
    #[cfg(not(feature = "board-rev-2"))]
    pub(super) w5500_intr: PA1<Input<Floating>>,

    pub(super) w5500: W5500<PB1<Output<PushPull>>>,

    /// EEPROM for configuration
    pub(super) ds93c46: DS93C46<PB0<Output<PushPull>>>,

    pub(super) spi: Spi<
        SPI1,
        Spi1NoRemap,
        (
            PA5<Alternate<PushPull>>,
            PA6<Input<Floating>>,
            PA7<Alternate<PushPull>>,
        ),
    >,

    #[allow(unused)]
    #[cfg(all(not(feature = "board-rev-2"), feature = "i2c2"))]
    pub(super) i2c: BlockingI2c<
        stm32f1xx_hal::pac::I2C2,
        (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>),
    >,

    /// Low-Active pin to restart the W5500
    pub(super) w5500_reset: PA4<Output<PushPull>>,

    #[cfg(feature = "board-rev-2")]
    pub(super) led_blue_handle_udp: PA3<Output<PushPull>>,

    #[cfg(feature = "board-rev-2")]
    pub(super) led_yellow_boot_warning: PA2<Output<PushPull>>,
    #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
    pub(super) led_yellow_boot_warning: PC14<Output<PushPull>>,
    #[cfg(all(not(feature = "board-rev-2"), not(feature = "i2c2")))]
    pub(super) led_yellow_boot_warning: PB11<Output<PushPull>>,

    #[cfg(feature = "board-rev-2")]
    pub(super) led_red_tmp_error: PA1<Output<PushPull>>,
    #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
    pub(super) led_red_tmp_error: PC15<Output<PushPull>>,
    #[cfg(all(not(feature = "board-rev-2"), not(feature = "i2c2")))]
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
}

impl Platform {
    pub fn save_network_configuration(&mut self) -> Result<(), ()> {
        match self
            .network_config
            .write(&mut self.ds93c46, &mut self.spi, &mut self.system.delay)
        {
            Err(_) => Err(()),
            Ok(_) => Ok(()),
        }
    }

    pub fn load_network_configuration(&mut self) -> Result<(), ()> {
        self.network_config
            .load(&mut self.ds93c46, &mut self.spi, &mut self.system.delay)
            .map(drop)
            .map_err(drop)
    }

    pub fn init_network(&mut self) -> Result<(), TransferError<SpiError, W5500CsError>> {
        // reset the network chip (timings are really generous)
        self.w5500_reset.set_low_infallible();
        self.system.delay.delay_ms(250_u16); // Datasheet 5.5.1 says 500_us to 1_ms (?)
        self.w5500_reset.set_high_infallible();
        self.system.delay.delay_ms(250_u16); // Datasheet says read RDY pin, the network chip needs some time to boot!

        let mut active = self.w5500.activate(&mut self.spi)?;
        let active = &mut active;

        // reset taken socket
        self.network_udp = None;

        // SAFETY: safe because take UDP socket is erased
        unsafe {
            active.reset()?;
        }

        self.system.delay.delay_ms(100_u16);

        active.set_mac(self.network_config.mac)?;
        active.set_ip(self.network_config.ip)?;
        active.set_subnet(self.network_config.subnet)?;
        active.set_gateway(self.network_config.gateway)?;

        self.system.delay.delay_ms(100_u16);

        if let Some(uninitialized) = active.take_socket(SOCKET_UDP) {
            // TODO lost on ERROR
            self.network_udp = (active, uninitialized)
                .try_into_udp_server_socket(SOCKET_UDP_PORT)
                .ok();
        }

        Ok(())
    }

    pub fn update(&mut self, module: &mut impl Module) {
        // do all the internal updates
        self.system.info.update_uptime_offset();
        self.check_factory_reset_flag();
        module.update(self);

        // check for external events
        if self.handle_udp_request(&mut [0u8; 2048], module).is_err() {
            self.log_error(ErrorFlags::NETWORK_UDP);
        } else if self.clear_error(ErrorFlags::NETWORK_UDP) {
            if self.init_network().is_ok() {
                self.clear_error(ErrorFlags::NETWORK_INIT);
            } else {
                self.log_error(ErrorFlags::NETWORK_INIT);
            }
        }

        // display results
        #[allow(unused)]
        let uptime_ms = self.system.info.uptime_ms();

        #[cfg(any(not(feature = "i2c2"), not(feature = "rtc")))]
        if let Some(err) = self.last_error_at_uptime_ms {
            if uptime_ms.saturating_sub(err) >= 1_000 {
                self.led_red_tmp_error.set_low_infallible();
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
                    self.send_udp(&ip, port, &response_buffer[..response_size])?;
                    false
                }
                Ok(Action::SendResponseAndReset) => {
                    self.send_udp(&ip, port, &response_buffer[..response_size])?;
                    true
                }
                Ok(Action::HandleRequest(request)) => {
                    Response::NotImplemented(request.id()).write(writer)?;
                    let response_size = available - writer.available();
                    self.send_udp(&ip, port, &response_buffer[..response_size])?;
                    false
                }
                Err(e) => {
                    let to_skip = available - writer.available();
                    Response::NotAvailable(request_id).write(writer)?;
                    let response_size = available - writer.available();
                    self.send_udp(&ip, port, &response_buffer[to_skip..response_size])?;
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
    ) -> Result<Option<(IpAddress, u16, usize)>, TransferError<SpiError, W5500CsError>> {
        let mut active = self.w5500.activate(&mut self.spi)?;
        let active = &mut active;
        let socket = self
            .network_udp
            .as_ref()
            .ok_or(TransferError::SpiError(spi::Error::ModeFault))?;

        (active, socket).receive(buffer)
    }

    pub fn send_udp(
        &mut self,
        host: &IpAddress,
        port: u16,
        data: &[u8],
    ) -> Result<(), TransferError<SpiError, W5500CsError>> {
        let mut active = self.w5500.activate(&mut self.spi)?;
        let socket = self
            .network_udp
            .as_ref()
            .ok_or(TransferError::SpiError(spi::Error::ModeFault))?;

        (&mut active, socket).blocking_send(host, port, data)
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
                self.network_config.mac.address.copy_from_slice(&mac);
                self.network_config.write(
                    &mut self.ds93c46,
                    &mut self.spi,
                    &mut self.system.delay,
                )?;
                Response::Ok(id, Format::Empty).write(response_writer)?;
                Ok(Action::SendResponseAndReset)
            }
            Request::SetNetworkIpSubnetGateway(id, ip, subnet, gateway) => {
                self.network_config.ip.address.copy_from_slice(&ip);
                self.network_config.subnet.address.copy_from_slice(&subnet);
                self.network_config
                    .gateway
                    .address
                    .copy_from_slice(&gateway);
                self.network_config.write(
                    &mut self.ds93c46,
                    &mut self.spi,
                    &mut self.system.delay,
                )?;
                Response::Ok(id, Format::Empty).write(response_writer)?;
                Ok(Action::SendResponseAndReset)
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
                    Format::ValueOnly(Type::Bytes((buffer.len() + name.len()) as u8)),
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
                Ok(Action::SendResponse)
            }
            Request::RetrieveNetworkConfiguration(id) => {
                Response::Ok(id, Format::ValueOnly(Type::Bytes(6 + 3 * 4)))
                    .write(response_writer)?;
                response_writer.write_all(&self.network_config.mac.address)?;
                response_writer.write_all(&self.network_config.ip.address)?;
                response_writer.write_all(&self.network_config.subnet.address)?;
                response_writer.write_all(&self.network_config.gateway.address)?;
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

        // TODO error handling
        if platform.load_network_configuration().is_err() {
            platform.log_error(ErrorFlags::NETWORK_LOAD);
            platform.log_boot_warning();
            for _ in 0..4 {
                platform.system.delay.delay_ms(1000_u16);
                platform.system.led_status.toggle_infallible();
            }
        }

        let _ = platform.init_onewire(); // allowed to fail

        if platform.init_network().is_err() {
            platform.log_error(ErrorFlags::NETWORK_INIT);
            platform.log_boot_warning();
        }

        let mut module = T::Builder::build(&mut platform, &mut constraints, module);

        platform.watchdog.start(5_000.ms());

        loop {
            for _ in 0..100 {
                platform.update(&mut module);
                platform.watchdog.feed();
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

            #[cfg(feature = "board-rev-2")]
            w5500_intr: gpiob.pb10.into_floating_input(&mut gpiob.crh),
            #[cfg(not(feature = "board-rev-2"))]
            w5500_intr: gpioa.pa1.into_floating_input(&mut gpioa.crl),
            w5500: {
                let mut w5500_cs = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

                w5500_cs.set_low_infallible(); // deselect

                w5500_reset.set_high_infallible(); // request reset
                delay.delay_ms(250_u16);
                w5500_reset.set_low_infallible(); // allow boot

                W5500::with_initialisation(
                    w5500_cs,
                    &mut spi,
                    OnWakeOnLan::Ignore,
                    OnPingRequest::Respond,
                    ConnectionType::Ethernet,
                    ArpResponses::Cache,
                )
                .unwrap()
            },

            spi,

            #[cfg(all(not(feature = "board-rev-2"), feature = "i2c2"))]
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

            ds93c46: {
                let mut ds93c46_cs = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
                ds93c46_cs.set_low_infallible(); // deselect
                DS93C46::new(ds93c46_cs)
            },

            w5500_reset,

            #[cfg(feature = "board-rev-2")]
            led_red_tmp_error: gpioa.pa1.into_push_pull_output(&mut gpioa.crl),
            #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
            led_red_tmp_error: gpioc.pc15.into_push_pull_output(&mut gpioc.crh),
            #[cfg(all(not(feature = "board-rev-2"), not(feature = "i2c2")))]
            led_red_tmp_error: gpiob.pb10.into_push_pull_output(&mut gpiob.crh),

            #[cfg(feature = "board-rev-2")]
            led_yellow_boot_warning: gpioa.pa2.into_push_pull_output(&mut gpioa.crl),
            #[cfg(all(not(feature = "board-rev-2"), not(feature = "rtc")))]
            led_yellow_boot_warning: gpioc.pc14.into_push_pull_output(&mut gpioc.crh),
            #[cfg(all(not(feature = "board-rev-2"), not(feature = "i2c2")))]
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

            watchdog: IndependentWatchdog::new(p.IWDG),
            system: {
                let timer = {
                    cp.DCB.enable_trace();
                    ::stm32f1xx_hal::time::MonoTimer::new(cp.DWT, clocks)
                };

                System {
                    info: DeviceInformation::new(&timer, cp.CPUID.base.read()),
                    delay,
                    timer,
                    led_status: gpioc.pc13.into_push_pull_output(&mut gpioc.crh),
                }
            },
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
        };

        let constraints = PlatformConstraints {
            afio,
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
    NetworkError(TransferError<spi::Error, Infallible>),
}

impl From<TransferError<spi::Error, Infallible>> for HandleError {
    fn from(e: TransferError<spi::Error, Infallible>) -> Self {
        HandleError::NetworkError(e)
    }
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
    }
}
