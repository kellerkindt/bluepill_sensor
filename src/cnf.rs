use crate::ds93c46::DS93C46;
use crate::platform::HandleError;
use core::convert::Infallible;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;
use onewire::compute_partial_crc8 as crc8;
use stm32f1xx_hal::spi;
use w5500::net::Ipv4Addr;
use w5500::MacAddress;

pub const MAGIC_EEPROM_CRC_START: u8 = 0x42;

pub struct NetworkConfiguration {
    pub mac: MacAddress,
    pub ip: Ipv4Addr,
    pub subnet: Ipv4Addr,
    pub gateway: Ipv4Addr,
}

impl NetworkConfiguration {
    pub const DEFAULT_MAC: MacAddress = MacAddress::new(0x02, 0x77, 0x00, 0x00, 0x01, 0x00);
    pub const DEFAULT_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 3, 225);
    pub const DEFAULT_SUBNET: Ipv4Addr = Ipv4Addr::new(255, 255, 255, 0);
    pub const DEFAULT_GATEWAY: Ipv4Addr = Ipv4Addr::new(192, 168, 3, 1);

    pub fn load(
        &mut self,
        eeprom: &mut DS93C46<impl OutputPin<Error = Infallible>>,
        spi: &mut impl FullDuplex<u8, Error = spi::Error>,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), HandleError> {
        let mut buf = [0u8; 6 + 3 * 4 + 2];
        eeprom.read(spi, delay, 0x00, &mut buf)?;
        if buf[0] != MAGIC_EEPROM_CRC_START {
            Err(HandleError::NotMagicCrcAtStart)
        } else {
            let crc = crc8(MAGIC_EEPROM_CRC_START, &buf[1..19]);
            if crc != buf[19] {
                Err(HandleError::CrcError)
            } else {
                self.mac = {
                    let mut octets = [0u8; 6];
                    octets.copy_from_slice(&buf[1..7]);
                    MacAddress::from(octets)
                };
                self.ip = {
                    let mut octets = [0u8; 4];
                    octets.copy_from_slice(&buf[7..11]);
                    Ipv4Addr::from(octets)
                };
                self.subnet = {
                    let mut octets = [0u8; 4];
                    octets.copy_from_slice(&buf[11..15]);
                    Ipv4Addr::from(octets)
                };
                self.gateway = {
                    let mut octets = [0u8; 4];
                    octets.copy_from_slice(&buf[15..19]);
                    Ipv4Addr::from(octets)
                };
                Ok(())
            }
        }
    }

    pub fn write(
        &self,
        eeprom: &mut DS93C46<impl OutputPin<Error = Infallible>>,
        spi: &mut impl FullDuplex<u8, Error = spi::Error>,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), HandleError> {
        let mut buf = [0u8; 6 + 3 * 4 + 2];
        buf[0] = MAGIC_EEPROM_CRC_START;
        buf[1..7].copy_from_slice(&self.mac.octets());
        buf[7..11].copy_from_slice(&self.ip.octets());
        buf[11..15].copy_from_slice(&self.subnet.octets());
        buf[15..19].copy_from_slice(&self.gateway.octets());
        let crc = onewire::compute_partial_crc8(MAGIC_EEPROM_CRC_START, &buf[1..19]);
        buf[19] = crc;
        eeprom.enable_write(spi, delay)?;
        let result = eeprom.write(spi, delay, 0x00, &buf);
        eeprom.disable_write(spi, delay)?;
        Ok(result?)
    }
}

impl Default for NetworkConfiguration {
    fn default() -> Self {
        NetworkConfiguration {
            mac: Self::DEFAULT_MAC,
            ip: Self::DEFAULT_IP,
            subnet: Self::DEFAULT_SUBNET,
            gateway: Self::DEFAULT_GATEWAY,
        }
    }
}
