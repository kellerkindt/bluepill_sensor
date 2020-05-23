use void::Void;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::Read;
use embedded_hal::blocking::i2c::Write;
use embedded_hal::blocking::i2c::WriteRead;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::serial::Read as SerialRead;
use embedded_hal::serial::Write as SerialWrite;
use embedded_hal::spi::FullDuplex;
use nb::Error as NbError;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::device::USART1;
use stm32f1xx_hal::gpio::gpioa::PA10;
use stm32f1xx_hal::gpio::gpioa::PA9;
use stm32f1xx_hal::gpio::Alternate;
use stm32f1xx_hal::gpio::Floating;
use stm32f1xx_hal::gpio::Input;
use stm32f1xx_hal::gpio::PushPull;
use stm32f1xx_hal::i2c::Error as I2cError;
use stm32f1xx_hal::serial::Error as SerialError;
use stm32f1xx_hal::serial::Serial;
use stm32f1xx_hal::spi;
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::time::Instant;
use stm32f1xx_hal::time::MonoTimer;

use crate::am2302::Am2302;
use crate::ds93c46::DS93C46;
use crate::NetworkConfiguration;
use core::convert::Infallible;
use onewire;
use onewire::ds18b20;
use onewire::OneWire;
use onewire::Sensor as OneWireSensor;
use w5500::*;

pub const SOCKET_UDP: Socket = Socket::Socket0;
pub const SOCKET_UDP_PORT: u16 = 51;

pub struct Platform<
    'a,
    CS: OutputPin<Error = Infallible>,
    Spi: FullDuplex<u8, Error = spi::Error>,
    NetworkReset: OutputPin<Error = Infallible>,
    ChipSelectEeprom: OutputPin<Error = Infallible>,
    PlatformReset: OutputPin<Error = Infallible>,
    OneWireOpenDrain: onewire::OpenDrainOutput<Error = Infallible>,
> {
    pub(super) information: DeviceInformation,

    // periphery
    pub(super) delay: &'a mut Delay,

    pub(super) onewire: OneWire<OneWireOpenDrain>,
    pub(super) spi: &'a mut Spi,
    pub(super) i2c: &'a mut WriteRead<Error = NbError<I2cError>>,
    pub(super) usart1_tx: &'a mut SerialWrite<u8, Error = Infallible>,
    pub(super) usart1_rx: &'a mut SerialRead<u8, Error = SerialError>,

    pub(super) network: &'a mut W5500<'a, CS>,
    pub(super) network_reset: NetworkReset,
    pub(super) network_config: NetworkConfiguration,
    pub(super) network_udp: Option<UdpSocket>,

    pub(super) humidity: [Am2302<'a>; 0],
    pub(super) eeprom: DS93C46<ChipSelectEeprom>,

    pub(super) reset: PlatformReset,
}

impl<
        'a,
        CS: OutputPin<Error = Infallible>,
        Spi: FullDuplex<u8, Error = spi::Error>,
        NetworkReset: OutputPin<Error = Infallible>,
        ChipSelectEeprom: OutputPin<Error = Infallible>,
        PlatformReset: OutputPin<Error = Infallible>,
        OneWireOpenDrain: onewire::OpenDrainOutput<Error = Infallible>,
    > Platform<'a, CS, Spi, NetworkReset, ChipSelectEeprom, PlatformReset, OneWireOpenDrain>
{
    pub fn save_network_configuration(&mut self) -> Result<(), ()> {
        match self
            .network_config
            .write(&mut self.eeprom, self.spi, self.delay)
        {
            Err(_) => Err(()),
            Ok(_) => Ok(()),
        }
    }

    pub fn load_network_configuration(&mut self) -> Result<(), ()> {
        match self
            .network_config
            .load(&mut self.eeprom, self.spi, self.delay)
        {
            Err(_) => Err(()),
            Ok(_) => Ok(()),
        }
    }

    pub fn init_network(&mut self) -> Result<(), TransferError<Spi::Error, CS::Error>> {
        let mut active = self.network.activate(self.spi)?;
        let active = &mut active;

        active.set_mac(self.network_config.mac)?;
        active.set_ip(self.network_config.ip)?;
        active.set_subnet(self.network_config.subnet)?;
        active.set_gateway(self.network_config.gateway)?;

        self.delay.delay_ms(10_u16);

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
        self.onewire.reset(self.delay).map(drop)
    }

    pub fn receive_udp(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<Option<(IpAddress, u16, usize)>, TransferError<Spi::Error, CS::Error>> {
        let mut active = self.network.activate(self.spi)?;
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
    ) -> Result<(), TransferError<spi::Error, CS::Error>> {
        let mut active = self.network.activate(self.spi)?;
        let socket = self
            .network_udp
            .as_ref()
            .ok_or(TransferError::SpiError(spi::Error::ModeFault))?;

        (&mut active, socket).blocking_send(host, port, data)
    }

    /// Discovers `onewire::Device`s on known `OneWire` bus's. Ignores faulty bus's.
    pub fn onewire_discover_devices<E, F: FnMut(onewire::Device) -> Result<bool, E>>(
        &mut self,
        mut f: F,
    ) -> Result<(), E> {
        let mut search = onewire::DeviceSearch::new();
        while let Ok(Some(device)) = self.onewire.search_next(&mut search, self.delay) {
            if !f(device)? {
                break;
            }
        }
        Ok(())
    }

    /// Prepares the given `onewire::Device` to be read. Returns the
    /// time in milliseconds the device needs until ready for read.
    /// Fails if the device is unsupported.
    pub fn onewire_prepare_read(&mut self, device: &onewire::Device) -> Result<u16, ()> {
        match device.family_code() {
            ds18b20::FAMILY_CODE => {
                if let Ok(dev) = ds18b20::DS18B20::new(device.clone()) {
                    if let Ok(time) = dev.start_measurement(&mut self.onewire, self.delay) {
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
                        match dev.read_temperature(&mut self.onewire, self.delay) {
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

    pub fn network_configuration(&self) -> &NetworkConfiguration {
        &self.network_config
    }

    pub fn network_configuration_mut(&mut self) -> &mut NetworkConfiguration {
        &mut self.network_config
    }

    pub fn reset(&mut self) {
        self.reset.set_low();
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

    pub fn uptime_ms(&self) -> u64 {
        self.uptime() / (self.frequency.0 / 1000) as u64
    }

    pub fn cpu_id(&self) -> u32 {
        self.cpuid
    }

    pub fn cpu_implementer(&self) -> u8 {
        (self.cpuid >> 24) as u8
    }

    pub fn cpu_variant(&self) -> u8 {
        (self.cpuid >> 20) as u8 & 0x0F
    }

    pub fn cpu_partnumber(&self) -> u16 {
        (self.cpuid >> 4) as u16 & 0x0F_FF
    }

    pub fn cpu_revision(&self) -> u8 {
        self.cpuid as u8 & 0x0F
    }
}

pub trait FullI2C: WriteRead + Write + Read {}

impl<T> FullI2C for T where
    T: WriteRead<Error = NbError<I2cError>>
        + Write<Error = NbError<I2cError>>
        + Read<Error = NbError<I2cError>>
{
}
