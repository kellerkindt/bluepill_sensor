use crate::ds93c46::DS93C46;
use crate::io_utils::OutputPinInfallible;
use crate::NetworkConfiguration;
use crate::{HandleError, System};
use byteorder::ByteOrder;
use byteorder::NetworkEndian;
use core::convert::Infallible;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::Read;
use embedded_hal::blocking::i2c::Write;
use embedded_hal::blocking::i2c::WriteRead;
use nb::Error as NbError;
use onewire;
use onewire::OneWire;
use onewire::Sensor as OneWireSensor;
use onewire::{ds18b20, DeviceSearch};
use sensor_common::Bus;
use sensor_common::Format;
use sensor_common::Request;
use sensor_common::Response;
use sensor_common::Type;
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
use stm32f1xx_hal::gpio::Input;
use stm32f1xx_hal::gpio::Output;
use stm32f1xx_hal::gpio::PushPull;
use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
use stm32f1xx_hal::i2c::Error as I2cError;
use stm32f1xx_hal::pac::SPI1;
use stm32f1xx_hal::spi;
use stm32f1xx_hal::spi::Spi;
use stm32f1xx_hal::spi::Spi1NoRemap;
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::time::Instant;
use stm32f1xx_hal::time::MonoTimer;
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
    /*
    pub(super) information: DeviceInformation,

    // periphery
    pub(super) delay: &'a mut Delay,

    pub(super) onewire: OneWire<OneWireOpenDrain>,
    pub(super) spi: &'a mut Spi,
    pub(super) usart1_tx: &'a mut SerialWrite<u8, Error = Infallible>,
    pub(super) usart1_rx: &'a mut SerialRead<u8, Error = SerialError>,

    pub(super) network: &'a mut W5500<'a, CS>,
    pub(super) network_reset: NetworkReset,
    pub(super) network_config: NetworkConfiguration,
    pub(super) network_udp: Option<UdpSocket>,

    pub(super) humidity: [Am2302<'a>; 0],
    pub(super) eeprom: DS93C46<ChipSelectEeprom>,

    pub(super) ltfm1: LongTimeFreqMeasurement,
    pub(super) ltfm2: LongTimeFreqMeasurement,
    pub(super) ltfm3: LongTimeFreqMeasurement,
    pub(super) ltfm4: LongTimeFreqMeasurement,
    */
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

    pub fn init_onewire(&mut self) -> Result<(), onewire::Error<Infallible>> {
        self.onewire.reset(&mut self.system.delay).map(drop)
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
                    crate::prepare_requested_on_one_wire(self, request_payload, response_writer)?;
                self.system.delay.delay_ms(ms_till_ready);
                crate::transmit_requested_on_one_wire(self, request_payload, response_writer)?;
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
                buffer[17] = crate::MAGIC_EEPROM_CRC_START;
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
            _ => Ok(Action::HandleRequest(request)),
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
