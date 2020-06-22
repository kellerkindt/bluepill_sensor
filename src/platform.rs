use crate::cnf::NetworkConfiguration;
use crate::ds93c46::DS93C46;
use crate::io_utils::OutputPinInfallible;
use crate::module::{ModulePeripherals, PlatformConstraints, RequestHandler};
use crate::{HandleError, System};
use byteorder::ByteOrder;
use byteorder::NetworkEndian;
use core::convert::Infallible;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::Read;
use embedded_hal::blocking::i2c::Write;
use embedded_hal::blocking::i2c::WriteRead;
use embedded_hal::spi::{Mode, Phase, Polarity};
use nb::Error as NbError;
use onewire;
use onewire::Sensor as OneWireSensor;
use onewire::{ds18b20, DeviceSearch};
use onewire::{Device, OneWire};
use sensor_common::Request;
use sensor_common::Response;
use sensor_common::Type;
use sensor_common::{Bus, Write as SensorWrite};
use sensor_common::{Format, Read as SensorRead};
use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::gpioa::PA0;
use stm32f1xx_hal::gpio::gpioa::PA1;
use stm32f1xx_hal::gpio::gpioa::PA2;
use stm32f1xx_hal::gpio::gpioa::PA3;
use stm32f1xx_hal::gpio::gpioa::PA4;
use stm32f1xx_hal::gpio::gpioa::PA5;
use stm32f1xx_hal::gpio::gpioa::PA6;
use stm32f1xx_hal::gpio::gpioa::PA7;
use stm32f1xx_hal::gpio::gpiob::PB0;
use stm32f1xx_hal::gpio::gpiob::PB1;
use stm32f1xx_hal::gpio::gpiob::PB10;
use stm32f1xx_hal::gpio::gpiob::PB11;
use stm32f1xx_hal::gpio::gpioc::PC15;
use stm32f1xx_hal::gpio::Floating;
use stm32f1xx_hal::gpio::GpioExt;
use stm32f1xx_hal::gpio::Input;
use stm32f1xx_hal::gpio::Output;
use stm32f1xx_hal::gpio::PushPull;
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
use stm32f1xx_hal::i2c::Error as I2cError;
use stm32f1xx_hal::pac::SPI1;
use stm32f1xx_hal::rcc::RccExt;
use stm32f1xx_hal::spi;
use stm32f1xx_hal::spi::Spi;
use stm32f1xx_hal::spi::Spi1NoRemap;
use stm32f1xx_hal::time::Instant;
use stm32f1xx_hal::time::MonoTimer;
use stm32f1xx_hal::time::{Hertz, U32Ext};
use w5500::*;

pub const SOCKET_UDP: Socket = Socket::Socket0;
pub const SOCKET_UDP_PORT: u16 = 51;

pub type SpiError = spi::Error;
pub type W5500CsError = Infallible;

/// Everything directly build onto the bluepill sensor board
pub struct Platform {
    pub(super) system: System,

    /// UserInput to restart the board
    pub(super) board_reset: PB11<Output<PushPull>>,

    /// W5500 interrupt pin
    #[allow(unused)]
    pub(super) w5500_intr: PB10<Input<Floating>>,
    pub(super) w5500: W5500<PB1<Output<PushPull>>>,

    /// ChipSelect for the EEPROM
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

    /// Low-Active pin to restart the W5500
    pub(super) w5500_reset: PA4<Output<PushPull>>,

    pub(super) led_blue: PA3<Output<PushPull>>,
    pub(super) led_yellow: PA2<Output<PushPull>>,
    pub(super) led_red: PA1<Output<PushPull>>,

    /// UserInput to reset the configuration to flash-default
    pub(super) factory_reset: PA0<Output<OpenDrain>>,

    pub(super) onewire: OneWire<PC15<Output<OpenDrain>>>,

    pub(super) network_udp: Option<UdpSocket>,
    pub(super) network_config: NetworkConfiguration,
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
        let mut active = self.w5500.activate(&mut self.spi)?;
        let active = &mut active;

        active.set_mac(self.network_config.mac)?;
        active.set_ip(self.network_config.ip)?;
        active.set_subnet(self.network_config.subnet)?;
        active.set_gateway(self.network_config.gateway)?;

        self.system.delay.delay_ms(10_u16);

        if self.network_udp.is_none() {
            if let Some(uninitialized) = active.take_socket(SOCKET_UDP) {
                // TODO lost on ERROR
                self.network_udp = (active, uninitialized)
                    .try_into_udp_server_socket(SOCKET_UDP_PORT)
                    .ok();
            }
        }

        Ok(())
    }

    pub fn update(&mut self) {
        self.system.info.update_uptime_offset();
    }

    pub fn init_onewire(&mut self) -> Result<(), onewire::Error<Infallible>> {
        self.onewire.reset(&mut self.system.delay).map(drop)
    }

    pub fn handle_udp_request(
        &mut self,
        buffer: &mut [u8],
        module: &mut impl RequestHandler,
    ) -> Result<(), HandleError> {
        if let Some((ip, port, size)) = self.receive_udp(buffer)? {
            self.led_red.set_high_infallible();
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

                self.led_blue.set_low_infallible();
                self.led_yellow.set_low_infallible();

                let action =
                    self.try_handle_request(request, &mut &*request_content_buffer, writer, module);

                (available - writer.available(), action, id)
            };

            let reset = match action {
                Ok(Action::SendResponse) => {
                    self.send_udp(&ip, port, &response_buffer[..response_size])?;
                    self.led_yellow.set_high_infallible();
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

    pub fn try_handle_request(
        &mut self,
        request: Request,
        request_payload: &mut impl sensor_common::Read,
        response_writer: &mut impl sensor_common::Write,
        module: &mut impl RequestHandler,
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
            Request::RetrieveDeviceInformation(id) => {
                let mut buffer = [0u8; 4 + 8 + 6 + 1];
                Response::Ok(id, Format::ValueOnly(Type::Bytes(buffer.len() as u8)))
                    .write(response_writer)?;
                NetworkEndian::write_u32(&mut buffer[0..], self.system.info.frequency().0);
                NetworkEndian::write_u64(&mut buffer[4..], self.system.info.uptime());

                buffer[12] = self.system.info.cpu_implementer();
                buffer[13] = self.system.info.cpu_variant();
                NetworkEndian::write_u16(&mut buffer[14..], self.system.info.cpu_partnumber());
                buffer[16] = self.system.info.cpu_revision();
                buffer[17] = crate::cnf::MAGIC_EEPROM_CRC_START;
                buffer[18] = 0x00;

                response_writer.write_all(&buffer)?;
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

    pub fn reset(&mut self) {
        self.board_reset.set_low_infallible()
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
    pub fn take() -> Option<Self> {
        let cp = cortex_m::Peripherals::take()?;
        let p = stm32f1xx_hal::pac::Peripherals::take()?;
        Some(PlatformBuilder::from((cp, p)))
    }

    pub fn split(self) -> (Platform, PlatformConstraints, ModulePeripherals) {
        (self.platform, self.constraints, self.module_peripherals)
    }
}

impl From<(cortex_m::Peripherals, stm32f1xx_hal::pac::Peripherals)> for PlatformBuilder {
    fn from((mut cp, p): (cortex_m::Peripherals, stm32f1xx_hal::pac::Peripherals)) -> Self {
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
            board_reset: {
                let mut self_reset = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);
                self_reset.set_high_infallible();
                self_reset
            },

            w5500_intr: gpiob.pb10.into_floating_input(&mut gpiob.crh),
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

            ds93c46: {
                let mut ds93c46_cs = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
                ds93c46_cs.set_low_infallible(); // deselect
                DS93C46::new(ds93c46_cs)
            },

            w5500_reset,

            led_red: gpioa.pa1.into_push_pull_output(&mut gpioa.crl),
            led_yellow: gpioa.pa2.into_push_pull_output(&mut gpioa.crl),
            led_blue: gpioa.pa3.into_push_pull_output(&mut gpioa.crl),

            factory_reset: gpioa.pa0.into_open_drain_output(&mut gpioa.crl),

            onewire: OneWire::new(gpioc.pc15.into_open_drain_output(&mut gpioc.crh), false),

            network_config: NetworkConfiguration::default(),
            network_udp: None,

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
