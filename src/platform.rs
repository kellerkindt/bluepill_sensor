
use stm32f103xx_hal::spi;
use stm32f103xx_hal::delay::Delay;

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::FullDuplex;
use embedded_hal::blocking::delay::DelayUs;

use NetworkConfiguration;

use w5500::W5500;
use w5500::Socket;
use w5500::Interrupt;
use w5500::IpAddress;
use w5500::MacAddress;

use onewire;
use onewire::OneWire;
use ds93c46::DS93C46;

pub const SOCKET_UDP      : Socket = Socket::Socket0;
pub const SOCKET_UDP_PORT : u16    = 51;

pub struct Platform<'a, 'inner: 'a> {
    // peripherie
    pub(super) delay: &'a mut Delay,

    pub(super) onewire: &'a mut OneWire<'inner>,
    pub(super) spi: &'a mut FullDuplex<u8, Error=spi::Error>,

    pub(super) network: &'a mut W5500<'inner>,
    pub(super) network_reset:  &'a mut OutputPin,
    pub(super) network_config: NetworkConfiguration,

    pub(super) eeprom: &'a mut DS93C46<'inner>,

    pub(super) reset: &'a mut OutputPin,
}

impl<'a, 'inner: 'a> Platform<'a, 'inner> {
    pub fn load_network_configuration(&mut self) -> Result<(), ()> {
        match self.network_config.load(
            self.eeprom,
            self.spi,
            self.delay
        ) {
            Err(_) => Err(()),
            Ok(_) => Ok(())
        }
    }

    pub fn init_network(&mut self) -> Result<(), spi::Error> {
        self.network.set_mac    (self.spi, &self.network_config.mac)?;
        self.network.set_ip     (self.spi, &self.network_config.ip)?;
        self.network.set_subnet (self.spi, &self.network_config.subnet)?;
        self.network.set_gateway(self.spi, &self.network_config.gateway)?;


        /*
        self.network.set_socket_interrupt_mask(self.spi, SOCKET_UDP, &[
            Interrupt::SendOk,
        ])?;*/
        self.network.reset_interrupt(self.spi, SOCKET_UDP, Interrupt::SendOk)?;
        self.network.listen_udp   (self.spi, SOCKET_UDP, SOCKET_UDP_PORT)?;
        Ok(())
    }

    pub fn init_onewire(&mut self) -> Result<(), onewire::Error> {
        self.onewire.reset(self.delay)?;
        Ok(())
    }

    pub fn receive_udp(&mut self, buffer: &mut [u8]) -> Result<Option<(IpAddress, u16, usize)>, spi::Error> {
        self.network.try_receive_udp(
            self.spi,
            SOCKET_UDP,
            buffer
        )
    }

    pub fn is_udp_interrupt_set(&mut self, interrupt: Interrupt) -> Result<bool, spi::Error> {
        self.network.is_interrupt_set(
            self.spi,
            SOCKET_UDP,
            interrupt,
        )
    }

    pub fn reset_udp_interrupt(&mut self, interrupt: Interrupt) -> Result<(), spi::Error> {
        self.network.reset_interrupt(
            self.spi,
            SOCKET_UDP,
            interrupt,
        )
    }

    pub fn send_udp(&mut self, host: &IpAddress, port: u16, data: &[u8]) -> Result<(), spi::Error> {
        self.network.send_udp(
            self.spi,
            SOCKET_UDP,
            SOCKET_UDP_PORT,
            host,
            port,
            data,
        )?;
        for _ in 0..0xFFFF {
            // wait until sent
            if self.is_udp_interrupt_set(Interrupt::SendOk)? {
                self.reset_udp_interrupt(Interrupt::SendOk)?;
                break;
            }
        }
        // restore listen state
        self.network.listen_udp(self.spi, SOCKET_UDP, SOCKET_UDP_PORT)
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