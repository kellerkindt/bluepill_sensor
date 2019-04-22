use crate::block_while;
use ads1x1x::{Ads1x1x, DataRate16Bit, SlaveAddr};
use bme280::BME280;
use byteorder::ByteOrder;
use byteorder::NetworkEndian;
use embedded_hal::adc::OneShot;
use embedded_hal::blocking::i2c::WriteRead;
use platform::Platform;
use sensor_common::Response;
use stm32f103xx_hal::i2c::Error as I2cError;
use I2CRefWrapper;

pub type TimeMillis = u64;

macro_rules! impl_fn_ads1115 {
    ($target:ident, $platform:ident, $name:ident, $channel:ident) => {{
        let info = &$platform.information;
        let i2c = &mut *$platform.i2c;

        let time = || info.uptime_ms();
        let timeout = time() + 500;
        let in_time = || timeout > time();

        block_while!(
            in_time(),
            Ads1x1x::new_ads1115(I2CRefWrapper(i2c), SlaveAddr::default())
                .read(&mut ads1x1x::channel::$channel)
        )
        .map_err(drop)
        .map(|value| {
            let value = value as f32;
            $target.$name.update(value, time());
            value
        })
    }};
}

#[derive(Default)]
pub struct Palt {
    pub co2: (TimeMillis, Option<f32>),
    pub humidity: (TimeMillis, Option<f32>),
    pub temperature: (TimeMillis, Option<f32>),
    pub pressure: (TimeMillis, Option<f32>),

    pub soil_moisture: (TimeMillis, Option<f32>),
    pub soil_ph: (TimeMillis, Option<f32>),
    pub water_flow: (TimeMillis, Option<f32>),
    pub brightness: (TimeMillis, Option<f32>),
}

impl Palt {
    pub fn tick(&mut self, platform: &mut Platform) {
        self.update_old(platform);
    }

    pub fn update_old(&mut self, platform: &mut Platform) {
        let now = platform.information.uptime_ms();

        macro_rules! update {
            ($target:ident, $update:ident, $max_age:expr) => {
                if self.$target.age(now) > $max_age {
                    let _ = self.$update(platform);
                    self.$target.touch(now);
                }
            };
        }

        update!(co2, update_co2, 1_000);
        update!(humidity, update_humidity, 1_000);
        update!(temperature, update_temperature, 1_000);
        update!(pressure, update_pressure, 1_000);
        update!(soil_moisture, update_soil_moisture, 1_000);
        update!(soil_ph, update_soil_ph, 1_000);
        update!(water_flow, update_water_flow, 1_000);
        update!(brightness, update_brightness, 1_000);
    }

    pub fn update_co2(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        const START_BYTE: u8 = 0xFF;
        const COMMAND: u8 = 0x86;

        let info = &platform.information;
        let tx = &mut platform.usart1_tx;
        let rx = &mut platform.usart1_rx;

        let time = || info.uptime_ms();
        let timeout = time() + 500;
        let in_time = || timeout > time();

        block_while!(in_time(), tx.write(START_BYTE))
            .and(block_while!(in_time(), tx.write(0x01)))
            .and(block_while!(in_time(), tx.write(COMMAND)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x00)))
            .and(block_while!(in_time(), tx.write(0x79)))
            .map_err(drop)
            .and_then(|_| {
                Ok([
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                    block_while!(in_time(), rx.read()).map_err(drop)?,
                ])
            })
            .and_then(|value| {
                // 0xFF - summed value of 1..=7 + 1
                let checksum: u8 = 0xFF_u8 + (!(&value[1..=7]).iter().sum::<u8>() + 1_u8) + 1_u8;

                if checksum == value[8] && value[0] == START_BYTE && value[1] == COMMAND {
                    let value = NetworkEndian::read_u16(&value[2..4]) as f32;
                    self.co2.update(value, time());
                    Ok(value)
                } else {
                    Err(())
                }
            })
    }

    pub fn update_pressure(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        self.update_pressure_humidity_temperature(platform)
            .map(|v| v.0)
    }

    pub fn update_humidity(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        self.update_pressure_humidity_temperature(platform)
            .map(|v| v.1)
    }

    pub fn update_temperature(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        self.update_pressure_humidity_temperature(platform)
            .map(|v| v.2)
    }

    fn update_pressure_humidity_temperature(
        &mut self,
        platform: &mut Platform,
    ) -> Result<(f32, f32, f32), ()> {
        let info = &platform.information;
        let time = || info.uptime_ms();
        let mut bme = BME280::new_primary(platform.i2c, platform.delay);

        bme.init()
            .and_then(|_| bme.measure())
            .map_err(drop)
            .map(|value| {
                self.pressure.update(value.pressure, time());
                self.temperature.update(value.temperature, time());
                self.humidity.update(value.humidity, time());
                (value.pressure, value.temperature, value.humidity)
            })
    }

    pub fn update_soil_moisture(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        impl_fn_ads1115!(self, platform, soil_moisture, SingleA0)
    }

    pub fn update_soil_ph(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        impl_fn_ads1115!(self, platform, soil_ph, SingleA1)
    }

    pub fn update_water_flow(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        impl_fn_ads1115!(self, platform, water_flow, SingleA2)
    }

    pub fn update_brightness(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        impl_fn_ads1115!(self, platform, brightness, SingleA3)
    }
}

pub trait TimedValue {
    fn age(&self, current: TimeMillis) -> TimeMillis;

    fn value(&self) -> Option<f32>;

    fn update(&mut self, value: f32, current: TimeMillis);

    fn touch(&mut self, current: TimeMillis);
}

impl TimedValue for (TimeMillis, Option<f32>) {
    fn age(&self, current: TimeMillis) -> TimeMillis {
        current.overflowing_sub(self.0).0
    }

    fn value(&self) -> Option<f32> {
        self.1
    }

    fn update(&mut self, value: f32, current: TimeMillis) {
        *self = (current, Some(value))
    }

    fn touch(&mut self, current: TimeMillis) {
        self.0 = current;
    }
}
