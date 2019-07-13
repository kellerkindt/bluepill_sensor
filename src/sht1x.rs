use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};

pub trait OpenDrainOutput: OutputPin + InputPin {}
impl<P: OutputPin + InputPin> OpenDrainOutput for P {}

pub struct Sht1x<'a> {
    data: &'a mut OpenDrainOutput,
    clock: &'a mut OpenDrainOutput,
}

impl<'a> Sht1x<'a>  {
    pub fn new(data: &'a mut OpenDrainOutput, clock: &'a mut OpenDrainOutput) -> Self {
        data.set_high();
        clock.set_high();
        Self {
            data,
            clock
        }
    }

    pub fn read_temperature(&mut self) -> Result<f32, Error> {
        self.send_command(0b0000_0011)?;
        self.wait_for_result()?;
        let value = self.read_u16();
        Ok((value as f32) * 0.01_f32 - 39.66_f32)
    }

    pub fn read_humidity(&mut self) -> Result<f32, Error> {
        self.send_command(0b0000_0101)?;
        self.wait_for_result()?;
        let value = self.read_u16() as f32;
        let linear = -2.0468_f32 + (0.0367_f32 * value) - (0.0000015955_f32 * value * value);
        let temp = self.read_temperature()?;
        Ok((temp - 25.0_f32) * (0.01_f32 + (0.00008_f32 * value)) + linear)
    }

    fn send_command(&mut self, command: u8) -> Result<(), Error> {

        self.data.set_high();
        self.clock.set_high();
        self.data.set_low();
        self.clock.set_low();
        self.clock.set_high();
        self.data.set_high();
        self.clock.set_low();

        self.write_bit(command & 0b1000_0000 > 0);
        self.write_bit(command & 0b0100_0000 > 0);
        self.write_bit(command & 0b0010_0000 > 0);
        self.write_bit(command & 0b0001_0000 > 0);
        self.write_bit(command & 0b0000_1000 > 0);
        self.write_bit(command & 0b0000_0100 > 0);
        self.write_bit(command & 0b0000_0010 > 0);
        self.write_bit(command & 0b0000_0001 > 0);

        self.data.set_high();
        self.clock.set_high();

        if self.data.is_high() {
            return Err(Error::MissingAck);
        }

        self.clock.set_low();

        if self.data.is_low() {
        //    return Err(Error::MissingAck);
        }

        Ok(())
    }

    fn write_bit(&mut self, bit: bool) {
        for _ in 0..1000 {
            if self.clock.is_low() && self.clock.is_high() {
                break; // never going to happen, just delay a bit
            }
        }
        self.clock.set_high();
        for _ in 0..1000 {
            if self.clock.is_low() && self.clock.is_high() {
                break; // never going to happen, just delay a bit
            }
        }

        if bit {
            self.data.set_high();
        } else {
            self.data.set_low();
        }

        self.clock.set_low();
    }

    fn wait_for_result(&mut self) -> Result<(), Error> {
        self.data.set_high();
        for _ in 0..1000 {
            if self.data.is_low() {
                return Ok(());
            }
        }
//        Err(Error::MissingAck)
            Ok(())
    }

    fn read_u16(&mut self) -> u16 {
        let mut value = 0_u16;

        self.data.set_high();

        value |= self.read_bit::<u16>() << 15;
        value |= self.read_bit::<u16>() << 14;
        value |= self.read_bit::<u16>() << 13;
        value |= self.read_bit::<u16>() << 12;
        value |= self.read_bit::<u16>() << 11;
        value |= self.read_bit::<u16>() << 10;
        value |= self.read_bit::<u16>() <<  9;
        value |= self.read_bit::<u16>() <<  8;

        // send ack
        self.clock.set_high();
        self.data.set_low(); // ack
        self.clock.set_low();

        self.data.set_high();

        value |= self.read_bit::<u16>() << 7;
        value |= self.read_bit::<u16>() << 6;
        value |= self.read_bit::<u16>() << 5;
        value |= self.read_bit::<u16>() << 4;
        value |= self.read_bit::<u16>() << 3;
        value |= self.read_bit::<u16>() << 2;
        value |= self.read_bit::<u16>() << 1;
        value |= self.read_bit::<u16>() << 0;

        // skip CRC
        self.clock.set_high();
        self.clock.set_low();

        value
    }

    fn read_bit<T>(&mut self) -> T where T: From<u8> {
        self.clock.set_high();
        for _ in 0..1000 {
            if self.clock.is_low() && self.clock.is_high() {
                break; // never going to happen, just delay a bit
            }
        }
        let value = if self.data.is_high() {
            T::from(0x01_u8)
        } else {
            T::from(0x00_u8)
        };
        self.clock.set_low();
        value
    }
}

pub enum Error {
    MissingAck
}
