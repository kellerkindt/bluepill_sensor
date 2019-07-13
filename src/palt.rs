use ads1x1x::{Ads1x1x, DataRate16Bit, SlaveAddr};
use bme280::BME280;
use byteorder::ByteOrder;
use byteorder::NetworkEndian;
use embedded_hal::adc::OneShot;
use embedded_hal::blocking::i2c::WriteRead;
use embedded_hal::digital::{OutputPin, StatefulOutputPin, ToggleableOutputPin};
use platform::Platform;
use sensor_common::Response;
use stm32f103xx_hal::i2c::Error as I2cError;
use I2CRefWrapper;
use sht1x::Sht1x;

pub type TimeMillis = u64;

macro_rules! impl_fn_ads1115 {
    ($target:ident, $platform:ident, $name:ident, $channel:ident) => {{
        let info = &$platform.information;
        let i2c = &mut *$platform.i2c;

        let time = || info.uptime_ms();
        let timeout = time() + 800;
        let in_time = || timeout > time();

        let mut ads = Ads1x1x::new_ads1115(I2CRefWrapper(i2c), SlaveAddr::default());

        block_while!(in_time(), ads.read(&mut ads1x1x::channel::$channel))
            .map_err(drop)
            .map(|value| {
                let value = value as f32;
                $target.$name.update(value, time());
                value
            })
    }};
}

pub struct Palt<'a, 'b: 'a> {
    pub co2: (TimeMillis, Option<f32>),
    pub humidity: (TimeMillis, Option<f32>),
    pub temperature: (TimeMillis, Option<f32>),
    pub pressure: (TimeMillis, Option<f32>),

    pub rain: (TimeMillis, Option<f32>),
    pub soil_ph: (TimeMillis, Option<f32>),
    pub water_flow: (TimeMillis, Option<f32>),
    pub brightness: (TimeMillis, Option<f32>),

    pub soil_humidity: (TimeMillis, Option<f32>),

    pub heater: (TimeMillis, &'a mut UsefulOutputPin),
    pub window: (TimeMillis, &'a mut UsefulOutputPin),
    pub lights: (TimeMillis, &'a mut UsefulOutputPin),
    pub valves: (TimeMillis, &'a mut UsefulOutputPin),

    pub ok_led: (TimeMillis, &'a mut UsefulOutputPin),

    pub sht1x: &'a mut Sht1x<'b>,
}

impl<'a, 'b: 'a> Palt<'a, 'b> {
    pub fn new(
        heater: &'a mut UsefulOutputPin,
        window: &'a mut UsefulOutputPin,
        lights: &'a mut UsefulOutputPin,
        valves: &'a mut UsefulOutputPin,
        ok_led: &'a mut UsefulOutputPin,
        sht1x: &'a mut Sht1x<'b>,
    ) -> Self {
        Self {
            co2: (0, None),
            humidity: (0, None),
            temperature: (0, None),
            pressure: (0, None),

            rain: (0, None),
            soil_ph: (0, None),
            water_flow: (0, None),
            brightness: (0, None),

            soil_humidity: (0, None),

            heater: (0, heater),
            window: (0, window),
            lights: (0, lights),
            valves: (0, valves),
            ok_led: (0, ok_led),

            sht1x,
        }
    }

    pub fn tick(&mut self, platform: &mut Platform) {
        self.update_old(platform);
        let now = platform.information.uptime_ms();

        if now > self.heater.0 + 1_000 {
            if let Some(temperature) = self.temperature.1 {
                if temperature < 19_f32 {
                    self.heater.1.set_low();
                } else if temperature > 24_f32 {
                    self.heater.1.set_high();
                }
            } else {
                self.heater.1.set_high();
            }
            self.heater.0 = now;
        }

        if now > self.window.0 + 1_000 {
            let mut reset = 0;
            if let Some(temperature) = self.temperature.1 {
                if temperature < 24_f32 {
                    self.window.1.set_high(); // close window
                } else if temperature > 30_f32 {
                    self.window.1.set_low(); // open window
                }
            } else {
                reset += 1;
            }
            if let Some(co2) = self.co2.1 {
                if co2 < 750_f32 {
                    self.window.1.set_low();
                } else if co2 > 800_f32 {
                    self.window.1.set_high();
                }
            } else {
                reset += 1;
            }
            if reset == 2 {
                self.window.1.set_high();
            }
            self.window.0 = now;
        }

        if now > self.lights.0 + 1_000 {
            if let Some(brightness) = self.brightness.1 {
                if brightness < 1000_f32 {
                    self.lights.1.set_low(); // turn on lights
                } else {
                    self.lights.1.set_high(); // turn lights off
                }
            } else {
                self.lights.1.set_high();
            }
            self.lights.0 = now;
        }

        if now > self.valves.0 + 1_000 {
            if let Some(soil_humidity) = self.soil_humidity.1 {
                if soil_humidity < 63_f32 {
                    self.valves.1.set_low();
                } else if soil_humidity > 67_f32 {
                    self.valves.1.set_high();
                }
            } else {
                self.valves.1.set_high();
            }
            self.valves.0 = now;
        }

        {
            let mut okay = true;
            if let Some(temperature) = self.temperature.1 {
                if temperature < 16_f32 || temperature > 30_f32 {
                    okay = false;
                }
            }
            if let Some(co2) = self.co2.1 {
                if co2 < 700_f32 {
                    okay = false;
                }
            }
            if let Some(soil_humidity) = self.soil_humidity.1 {
                if soil_humidity > 70_f32 || soil_humidity < 60_f32 {
                    okay = false;
                }
            }
            if okay {
                self.ok_led.1.set_low();
            } else {
                self.ok_led.1.set_high();
            }
            self.ok_led.0 = now;
        }
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
        update!(rain, update_rain, 1_000);
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
                (value.pressure, value.humidity, value.temperature)
            })
    }

    pub fn update_rain(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        // impl_fn_ads1115!(self, platform, rain, SingleA1)
        self.rain.0 = platform.information.uptime_ms();
        self.rain.1 = Some(0_f32);
        Ok(0_f32)
    }

    pub fn update_soil_ph(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        impl_fn_ads1115!(self, platform, soil_ph, SingleA2)
    }

    pub fn update_water_flow(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        // impl_fn_ads1115!(self, platform, water_flow, SingleA0)

        let now = platform.information.uptime_ms();

        let flow = if self.valves.1.is_set_low() {
            24_f32 + ((now % 100) as f32 / 50_f32)
        } else {
            0_f32
        };


        self.water_flow.0 = now;
        self.water_flow.1 = Some(flow);
        Ok(flow)
    }

    pub fn update_brightness(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        impl_fn_ads1115!(self, platform, brightness, SingleA3)
    }

    pub fn update_soil_humidity(&mut self, platform: &mut Platform) -> Result<f32, ()> {
        impl_fn_ads1115!(self, platform, brightness, SingleA1)
        /*let _ = platform;
        self.soil_humidity.0 = platform.information.uptime_ms();
        self.soil_humidity.1 = Some(0_f32);
        Ok(0_f32)*/
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

pub trait UsefulOutputPin: StatefulOutputPin + OutputPin {

}

impl<P: StatefulOutputPin + OutputPin> UsefulOutputPin for P {}

pub trait DriverPin {
    fn force_set(&mut self, now: TimeMillis, value: f32);
}

impl<'a> DriverPin for (TimeMillis, &'a mut UsefulOutputPin) {
    fn force_set(&mut self, now: u64, value: f32) {
        self.0 = now + 60_000; // enforce forced state for a bit longer
        if value > 0_f32 {
            self.1.set_low();
        } else {
            self.1.set_high();
        }
    }
}
