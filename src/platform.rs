use void::Void;

use stm32f103xx_hal::delay::Delay;
use stm32f103xx_hal::spi;
use stm32f103xx_hal::time::Hertz;
use stm32f103xx_hal::time::Instant;
use stm32f103xx_hal::time::MonoTimer;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::Read;
use embedded_hal::blocking::i2c::Write;
use embedded_hal::blocking::i2c::WriteRead;
use embedded_hal::digital::OutputPin;
use embedded_hal::serial::Read as SerialRead;
use embedded_hal::serial::Write as SerialWrite;
use embedded_hal::spi::FullDuplex;

use nb::Error as NbError;
use stm32f103xx_hal::device::USART1;
use stm32f103xx_hal::gpio::gpioa::PA10;
use stm32f103xx_hal::gpio::gpioa::PA9;
use stm32f103xx_hal::gpio::Alternate;
use stm32f103xx_hal::gpio::Floating;
use stm32f103xx_hal::gpio::Input;
use stm32f103xx_hal::gpio::PushPull;
use stm32f103xx_hal::i2c::Error as I2cError;
use stm32f103xx_hal::serial::Error as SerialError;
use stm32f103xx_hal::serial::Serial;

use NetworkConfiguration;

use ds18b20;

use am2302::Am2302;

use w5500::*;

use ds93c46::DS93C46;
use onewire;
use onewire::OneWire;
use onewire::Sensor as OneWireSensor;

pub const SOCKET_UDP: Socket = Socket::Socket0;
pub const SOCKET_UDP_PORT: u16 = 51;

pub struct Platform<'a, 'inner: 'a> {
    pub(super) information: DeviceInformation,

    // periphery
    pub(super) delay: &'a mut Delay,

    pub(super) onewire: &'a mut [&'a mut OneWire<'inner>],
    pub(super) spi: &'a mut FullDuplex<u8, Error = spi::Error>,
    pub(super) i2c: &'a mut WriteRead<Error = NbError<I2cError>>,
    pub(super) usart1_tx: &'a mut SerialWrite<u8, Error = Void>,
    pub(super) usart1_rx: &'a mut SerialRead<u8, Error = SerialError>,

    pub(super) network: &'a mut W5500<'inner>,
    pub(super) network_reset: &'a mut OutputPin,
    pub(super) network_config: NetworkConfiguration,
    pub(super) network_udp: Option<UdpSocket>,

    pub(super) humidity: [Am2302<'a>; 5],
    pub(super) eeprom: &'a mut DS93C46<'inner>,

    pub(super) reset: &'a mut OutputPin,
}

impl<'a, 'inner: 'a> Platform<'a, 'inner> {
    pub fn save_network_configuration(&mut self) -> Result<(), ()> {
        match self.network_config.write(self.eeprom, self.spi, self.delay) {
            Err(_) => Err(()),
            Ok(_) => Ok(()),
        }
    }

    pub fn load_network_configuration(&mut self) -> Result<(), ()> {
        match self.network_config.load(self.eeprom, self.spi, self.delay) {
            Err(_) => Err(()),
            Ok(_) => Ok(()),
        }
    }

    pub fn init_network(&mut self) -> Result<(), spi::Error> {
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

    pub fn init_onewire(&mut self) -> Result<(), onewire::Error> {
        let mut result = Ok(());
        for w in self.onewire.iter_mut() {
            if let Err(e) = w.reset(self.delay) {
                result = Err(e);
            }
        }
        result
    }

    pub fn receive_udp(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<Option<(IpAddress, u16, usize)>, spi::Error> {
        let mut active = self.network.activate(self.spi)?;
        let active = &mut active;
        let socket = self.network_udp.as_ref().ok_or(spi::Error::ModeFault)?;

        (active, socket).receive(buffer)
    }

    pub fn send_udp(&mut self, host: &IpAddress, port: u16, data: &[u8]) -> Result<(), spi::Error> {
        let mut active = self.network.activate(self.spi)?;
        let active = &mut active;
        let socket = self.network_udp.as_ref().ok_or(spi::Error::ModeFault)?;

        (active, socket).blocking_send(host, port, data)
    }

    /// Discovers `onewire::Device`s on known `OneWire` bus's. Ignores faulty bus's.
    pub fn onewire_discover_devices<E, F: FnMut(onewire::Device) -> Result<bool, E>>(
        &mut self,
        mut f: F,
    ) -> Result<(), E> {
        'outer: for wire in self.onewire.iter_mut() {
            let mut search = onewire::DeviceSearch::new();
            while let Ok(Some(device)) = wire.search_next(&mut search, self.delay) {
                if !f(device)? {
                    break 'outer;
                }
            }
        }
        Ok(())
    }

    /// Prepares the given `onewire::Device` to be read. Returns the
    /// time in milliseconds the device needs until ready for read.
    /// Fails if the device is unsupported.
    pub fn onewire_prepare_read(&mut self, device: &onewire::Device) -> Result<u16, ()> {
        let mut result = Err(());
        match device.family_code() {
            ds18b20::FAMILY_CODE => {
                if let Ok(dev) = ds18b20::DS18B20::new(device.clone()) {
                    for wire in self.onewire.iter_mut() {
                        if let Ok(time) = dev.start_measurement(wire, self.delay) {
                            result = Ok(time);
                            break;
                        }
                    }
                }
            }
            _ => {}
        }
        result
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
                    for wire in self.onewire.iter_mut() {
                        for _ in 0..retry_count_on_crc_error {
                            match dev.read_temperature(wire, self.delay) {
                                Ok(value) => return Ok(value as f32),
                                Err(onewire::Error::CrcMismatch(_, _)) => continue,
                                _ => break,
                            }
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
