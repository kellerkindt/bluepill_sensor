use crate::io_utils::OutputPinInfallible;
use core::convert::Infallible;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;

pub struct DS93C46<ChipSelect: OutputPin<Error = Infallible>> {
    cs: ChipSelect,
}

impl<ChipSelect: OutputPin<Error = Infallible>> DS93C46<ChipSelect> {
    pub fn new(cs: ChipSelect) -> Self {
        DS93C46 { cs }
    }
    pub fn enable_write<E>(
        &mut self,
        spi: &mut impl FullDuplex<u8, Error = E>,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), E> {
        self.select_chip();
        let result_1 = Self::write_byte(spi, 0x02);
        let result = Self::write_byte(spi, 0x60);
        self.deselect_chip();
        delay.delay_ms(10);
        result_1?;
        result
    }

    pub fn disable_write<E>(
        &mut self,
        spi: &mut impl FullDuplex<u8, Error = E>,
        delay: &mut impl DelayMs<u16>,
    ) -> Result<(), E> {
        self.select_chip();
        let result_1 = Self::write_byte(spi, 0x02);
        let result = Self::write_byte(spi, 0x00);
        self.deselect_chip();
        delay.delay_ms(10);
        result_1?;
        result
    }

    pub fn write<E>(
        &mut self,
        spi: &mut impl FullDuplex<u8, Error = E>,
        delay: &mut impl DelayMs<u16>,
        address: u8,
        content: &[u8],
    ) -> Result<(), E> {
        let result = self.write_raw(spi, delay, address, content);
        self.deselect_chip();
        result
    }

    fn write_raw<E>(
        &mut self,
        spi: &mut impl FullDuplex<u8, Error = E>,
        delay: &mut impl DelayMs<u16>,
        address: u8,
        content: &[u8],
    ) -> Result<(), E> {
        for i in 0..content.len() {
            self.select_chip();
            Self::write_byte(spi, 0x02)?;
            Self::write_byte(spi, 0x80 | (address + i as u8))?;
            Self::write_byte(spi, content[i])?;
            self.deselect_chip();
            delay.delay_ms(10);
        }
        Ok(())
    }

    pub fn read<E>(
        &mut self,
        spi: &mut impl FullDuplex<u8, Error = E>,
        delay: &mut impl DelayMs<u16>,
        address: u8,
        target: &mut [u8],
    ) -> Result<(), E> {
        let result = self.read_raw(spi, delay, address, target);
        self.deselect_chip();
        result
    }

    fn read_raw<E>(
        &mut self,
        spi: &mut impl FullDuplex<u8, Error = E>,
        _delay: &mut impl DelayMs<u16>,
        address: u8,
        target: &mut [u8],
    ) -> Result<(), E> {
        for i in 0..target.len() {
            self.select_chip();
            Self::write_byte(spi, 0x03)?;
            Self::write_byte(spi, !0x80 & (address + i as u8))?;
            //delay.delay_ms(5);
            let b1 = Self::read_byte(spi)?;
            let b2 = Self::read_byte(spi)?;
            self.deselect_chip();
            target[i] = b1 << 1 | b2 >> 7;
            // delay.delay_us(100);
        }
        Ok(())
    }

    fn read_byte<E>(spi: &mut impl FullDuplex<u8, Error = E>) -> Result<u8, E> {
        block!(spi.send(0x00))?;
        block!(spi.read())
    }

    fn write_byte<E>(spi: &mut impl FullDuplex<u8, Error = E>, byte: u8) -> Result<(), E> {
        block!(spi.send(byte))?;
        block!(spi.read())?;
        Ok(())
    }

    fn select_chip(&mut self) {
        self.cs.set_high_infallible();
    }

    fn deselect_chip(&mut self) {
        self.cs.set_low_infallible();
    }
}
