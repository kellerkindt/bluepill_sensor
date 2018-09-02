use stm32f103xx_hal::delay::Delay;
use stm32f103xx_hal::spi;
use stm32f103xx_hal::time::Hertz;
use stm32f103xx_hal::time::Instant;
use stm32f103xx_hal::time::MonoTimer;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::FullDuplex;

use NetworkConfiguration;

use am2302::Am2302;

use w5500::Interrupt;
use w5500::IpAddress;
use w5500::Socket;
use w5500::W5500;

use ds93c46::DS93C46;
use onewire;
use onewire::OneWire;

pub const SOCKET_UDP: Socket = Socket::Socket0;
pub const SOCKET_UDP_PORT: u16 = 51;

pub struct Platform<'a, 'inner: 'a> {
    pub(super) information: DeviceInformation,

    // peripherie
    pub(super) delay: &'a mut Delay,

    pub(super) onewire: &'a mut [&'a mut OneWire<'inner>],
    pub(super) spi: &'a mut FullDuplex<u8, Error = spi::Error>,

    pub(super) network: &'a mut W5500<'inner>,
    pub(super) network_reset: &'a mut OutputPin,
    pub(super) network_config: NetworkConfiguration,

    pub(super) humidity: [Am2302<'a>; 7],
    pub(super) eeprom: &'a mut DS93C46<'inner>,

    pub(super) reset: &'a mut OutputPin,
}

impl<'a, 'inner: 'a> Platform<'a, 'inner> {
    pub fn save_network_configuration(&mut self) -> Result<(), ()> {
        match self.network_config.write(self.eeprom, self.spi, self.delay) {
            Err(_) => Err(()),
            Ok(_) => Ok(())
        }
    }

    pub fn load_network_configuration(&mut self) -> Result<(), ()> {
        match self.network_config.load(self.eeprom, self.spi, self.delay) {
            Err(_) => Err(()),
            Ok(_) => Ok(()),
        }
    }

    pub fn init_network(&mut self) -> Result<(), spi::Error> {
        self.network.init(self.spi)?;
        self.network.set_mac(self.spi, &self.network_config.mac)?;
        self.network.set_ip(self.spi, &self.network_config.ip)?;
        self.network
            .set_subnet(self.spi, &self.network_config.subnet)?;
        self.network
            .set_gateway(self.spi, &self.network_config.gateway)?;

        self.delay.delay_ms(10_u16);

        /*
        self.network.set_socket_interrupt_mask(self.spi, SOCKET_UDP, &[
            Interrupt::SendOk,
        ])?;*/
        self.network
            .reset_interrupt(self.spi, SOCKET_UDP, Interrupt::SendOk)?;
        self.network
            .listen_udp(self.spi, SOCKET_UDP, SOCKET_UDP_PORT)?;
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
        self.network.try_receive_udp(self.spi, SOCKET_UDP, buffer)
    }

    pub fn is_udp_interrupt_set(&mut self, interrupt: Interrupt) -> Result<bool, spi::Error> {
        self.network
            .is_interrupt_set(self.spi, SOCKET_UDP, interrupt)
    }

    pub fn reset_udp_interrupt(&mut self, interrupt: Interrupt) -> Result<(), spi::Error> {
        self.network
            .reset_interrupt(self.spi, SOCKET_UDP, interrupt)
    }

    pub fn send_udp(&mut self, host: &IpAddress, port: u16, data: &[u8]) -> Result<(), spi::Error> {
        self.network
            .send_udp(self.spi, SOCKET_UDP, SOCKET_UDP_PORT, host, port, data)?;
        for _ in 0..0xFFFF {
            // wait until sent
            if self.is_udp_interrupt_set(Interrupt::SendOk)? {
                self.reset_udp_interrupt(Interrupt::SendOk)?;
                break;
            }
        }
        // restore listen state
        self.network
            .listen_udp(self.spi, SOCKET_UDP, SOCKET_UDP_PORT)
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
