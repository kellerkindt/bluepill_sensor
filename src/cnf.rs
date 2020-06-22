use crate::ds93c46::DS93C46;
use crate::HandleError;
use core::convert::Infallible;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;
use onewire::compute_partial_crc8 as crc8;
use stm32f1xx_hal::spi;
use w5500::{IpAddress, MacAddress};

pub const MAGIC_EEPROM_CRC_START: u8 = 0x42;

pub struct NetworkConfiguration {
    pub mac: MacAddress,
    pub ip: IpAddress,
    pub subnet: IpAddress,
    pub gateway: IpAddress,
}

impl NetworkConfiguration {
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
                self.mac.address.copy_from_slice(&buf[1..7]);
                self.ip.address.copy_from_slice(&buf[7..11]);
                self.subnet.address.copy_from_slice(&buf[11..15]);
                self.gateway.address.copy_from_slice(&buf[15..19]);
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
        buf[1..7].copy_from_slice(&self.mac.address);
        buf[7..11].copy_from_slice(&self.ip.address);
        buf[11..15].copy_from_slice(&self.subnet.address);
        buf[15..19].copy_from_slice(&self.gateway.address);
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
            mac: MacAddress::new(0x02, 0x00, 0x00, 0x00, 0x01, 0x00),
            ip: IpAddress::new(192, 168, 3, 224),
            subnet: IpAddress::new(255, 255, 255, 0),
            gateway: IpAddress::new(192, 168, 3, 1),
        }
    }
}
