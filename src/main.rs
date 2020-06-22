#![no_std]
#![no_main]
#![deny(intra_doc_link_resolution_failure)]

extern crate cortex_m;
//#[macro_use(entry, exception)]
#[macro_use(entry)]
extern crate cortex_m_rt;
//extern crate panic_abort;
extern crate panic_halt;

extern crate embedded_hal;
extern crate stm32f1xx_hal;

extern crate byteorder;
extern crate void;

//#[macro_use(block_while)]
#[macro_use(block)]
extern crate nb;

extern crate ads1x1x;
extern crate onewire;
// extern crate pcd8544;
extern crate sensor_common;
extern crate w5500;

// mod am2302;
mod ds93c46;
mod io_utils;
mod module;
mod platform;
mod system;
// mod sht1x;
mod cnf;

use crate::cnf::NetworkConfiguration;
use crate::io_utils::*;
use crate::module::ecm::{ElectricCounterModule, LongTimeFreqMeasurement};
use crate::module::Module;
use crate::system::System;
use core::convert::Infallible;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::InputPin;
use nb::Error as NbError;
use platform::*;
use stm32f1xx_hal::i2c::Error as I2cError;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::spi;
use w5500::*;

/*
#[macro_export]
macro_rules! block_while {
    ($c:expr, $e:expr) => {
        loop {
            #[allow(unreachable_patterns)]
            match $e {
                Err(nb::Error::Other(e)) => {
                    #[allow(unreachable_code)]
                    break Err(nb::Error::Other(e))
                }
                Err(nb::Error::WouldBlock) => {
                    if !$c {
                        break Err(nb::Error::WouldBlock);
                    }
                }
                Ok(x) => break Ok(x),
            }
        }
    };
}
*/

#[entry]
fn main() -> ! {
    let (mut platform, mut constraints, module) = PlatformBuilder::take().unwrap().split();

    // TODO error handling
    if platform.load_network_configuration().is_err() {
        for _ in 0..4 {
            platform.system.delay.delay_ms(1000_u16);
            if platform.system.led_status.is_set_low().unwrap() {
                platform.system.led_status.set_high_infallible();
            } else {
                platform.system.led_status.set_low_infallible();
            }
        }
    }

    platform.w5500_reset.set_low_infallible();
    platform.system.delay.delay_ms(250_u16);
    platform.w5500_reset.set_high_infallible();
    platform.system.delay.delay_ms(25_u16); // the network chip needs some time to boot!

    let _ = platform.init_network();
    let _ = platform.init_onewire();

    let mut tick = 0_u64;

    let (pa15, pb3, pb4) =
        constraints
            .afio
            .mapr
            .disable_jtag(module.pin_38, module.pin_39, module.pin_40);

    let mut ecc = ElectricCounterModule {
        pa8: module.pin_29.into_pull_up_input(&mut constraints.gpioa_crh),
        pa12: module
            .pin_33
            .into_floating_input(&mut constraints.gpioa_crh),
        pa15: pa15.into_floating_input(&mut constraints.gpioa_crh),
        pb3: pb3.into_floating_input(&mut constraints.gpiob_crl),
        pb4: pb4.into_floating_input(&mut constraints.gpiob_crl),

        garage_open_since: None,
        ltfm1: LongTimeFreqMeasurement::new(),
        ltfm2: LongTimeFreqMeasurement::new(),
        ltfm3: LongTimeFreqMeasurement::new(),
        ltfm4: LongTimeFreqMeasurement::new(),
    };

    loop {
        platform.update();
        ecc.update(&mut platform);

        if tick % 100 == 0 {
            if platform.system.led_status.is_set_low().unwrap_or(false) {
                platform.system.led_status.set_high_infallible();
            } else {
                platform.system.led_status.set_low_infallible();
            }
        }

        match platform.handle_udp_request(
            &mut [0u8; 2048],
            &mut ecc, // Option::<&mut Infallible>::None,
        ) {
            Err(_e) => {
                // writeln!(display, "Error:");
                // writeln!(display, "{:?}", e);
                platform.led_yellow.set_high_infallible();
                platform.system.delay.delay_ms(1_000_u16);
            }
            Ok(_address) => {}
        };

        {
            let probe_start = platform.system.timer.now();
            while platform.factory_reset.is_high().unwrap_or(false) {
                // pressed for longer than 3s?
                if (probe_start.elapsed() / platform.system.timer.frequency().0) > 3 {
                    platform.network_config = NetworkConfiguration::default();
                    let _ = platform.save_network_configuration();
                    while platform.factory_reset.is_high().unwrap_or(false) {
                        platform.led_blue.set_high_infallible();
                        platform.led_yellow.set_high_infallible();
                        platform.led_red.set_high_infallible();
                    }
                    platform.reset();
                }
            }
        }

        tick += 1;
    }
}

#[derive(Debug)]
pub enum HandleError {
    Unknown,
    Spi(spi::Error),
    Parsing(sensor_common::Error),
    OneWire(onewire::Error<Infallible>),
    Unavailable,
    NotMagicCrcAtStart,
    CrcError,
    I2c(NbError<I2cError>),
    NetworkError(TransferError<spi::Error, Infallible>),
}

impl From<TransferError<spi::Error, Infallible>> for HandleError {
    fn from(e: TransferError<spi::Error, Infallible>) -> Self {
        HandleError::NetworkError(e)
    }
}

impl From<spi::Error> for HandleError {
    fn from(e: spi::Error) -> Self {
        HandleError::Spi(e)
    }
}

impl From<sensor_common::Error> for HandleError {
    fn from(e: sensor_common::Error) -> Self {
        HandleError::Parsing(e)
    }
}

impl From<onewire::Error<Infallible>> for HandleError {
    fn from(e: onewire::Error<Infallible>) -> Self {
        HandleError::OneWire(e)
    }
}

impl From<()> for HandleError {
    fn from(_: ()) -> Self {
        HandleError::Unknown
    }
}

impl From<NbError<I2cError>> for HandleError {
    fn from(e: NbError<I2cError>) -> Self {
        HandleError::I2c(e)
    }
}
