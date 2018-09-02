use core::f32;

use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;

use stm32f103xx_hal::time::MonoTimer;

pub trait OpenDrainOutput: OutputPin + InputPin {}
impl<P: OutputPin + InputPin> OpenDrainOutput for P {}

pub struct Am2302<'a> {
    pin: &'a mut OpenDrainOutput,
    timer: &'a MonoTimer,
}

impl<'a> Am2302<'a> {
    pub fn new(pin: &'a mut OpenDrainOutput, timer: &'a MonoTimer) -> Am2302<'a> {
        pin.set_high();
        Am2302 { pin, timer }
    }

    pub fn read(&mut self) -> Result<ReadValue, Error> {
        match self.read_set() {
            Ok(raw) => {
                self.pin.set_high();
                Ok(Self::parse(raw)?)
            }
            Err(e) => {
                self.pin.set_high();
                Err(e)
            }
        }
    }

    fn parse(buffer: [u8; 5]) -> Result<ReadValue, Error> {
        if buffer[0] + buffer[1] + buffer[2] + buffer[3] != buffer[4] {
            return Err(Error::Crc);
        }
        Ok(ReadValue {
            humidity: ((buffer[0] as u16) << 8 | buffer[1] as u16) as f32 * 0.1_f32,
            temperature: ((buffer[2] as u16) << 8 | buffer[3] as u16) as f32 * 0.1_f32,
        })
    }

    fn read_set(&mut self) -> Result<[u8; 5], Error> {
        self.pin.set_low();
        //delay.delay_us(5_000);
        let start = self.timer.now();
        // pull low 'beyond at least 1~10ms'
        while start.elapsed() < 5_000 as u32 * (self.timer.frequency().0 / 1_000_000) {}
        self.pin.set_high();

        // sensor pulls down after 20-40us
        self.expect_low_within(45)?;

        // sensor should pull up after 80us
        self.expect_high_within(85)?;

        // sensor should pull low after 80us
        self.expect_low_within(85)?;

        let mut buffer = [0u8; 5];

        for i in 0..buffer.len() * 8 {
            // sensor should keep low for 50us
            self.expect_high_within(70)?;

            // if sensor pulls low after 26us-28us it sends a 0 bit
            if self.expect_low_within(30).is_ok() {
                // 0 bit
            } else {
                // not low, so a 1 bit is sent
                buffer[i / 8] |= (1 << 7 - (i % 8)) as u8;
                // high in total 70us
                self.expect_low_within(40)?;
            }
        }
        Ok(buffer)
    }

    fn expect_high_within(&self, timeout: u16) -> Result<(), Error> {
        if self.wait_for(true, timeout) {
            Ok(())
        } else {
            Err(Error::NotHighWithin(timeout))
        }
    }

    fn expect_low_within(&self, timeout: u16) -> Result<(), Error> {
        if self.wait_for(false, timeout) {
            Ok(())
        } else {
            Err(Error::NotLowWithin(timeout))
        }
    }

    fn wait_for(&self, state: bool, timeout: u16) -> bool {
        // 1us is nearly nothing for a 8MHz chip (8 clock ticks = 1us),
        // so step in 10us to reduce relative overhead an keep somehow
        // near the requested timeout
        // TODO MonoTimer?
        let now = self.timer.now();
        while now.elapsed() <= timeout as u32 * (self.timer.frequency().0 / 1_000_000) {
            if self.pin.is_high() == state {
                return true;
            }
        }
        return false;
    }
}

pub struct ReadValue {
    pub humidity: f32,
    pub temperature: f32,
}

pub enum Error {
    Crc,
    NotHighWithin(u16),
    NotLowWithin(u16),
}
