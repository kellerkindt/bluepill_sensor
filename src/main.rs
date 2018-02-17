//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---

#![feature(used)]
#![no_std]


extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;

extern crate stm32f103xx;
extern crate stm32f103xx_hal;

extern crate onewire;


use stm32f103xx_hal::prelude::*;
use stm32f103xx_hal::prelude::_embedded_hal_digital_OutputPin as OutputPin;
use stm32f103xx_hal::gpio::Output;
use stm32f103xx_hal::gpio::OpenDrain;
use stm32f103xx_hal::gpio::gpioc::PCx;
use stm32f103xx_hal::delay::Delay;

use stm32f103xx::GPIOC;

use core::fmt::Write;

use cortex_m::asm;
use self::cortex_m_semihosting::hio;

use onewire::*;



fn main() {
    stdout(|out| writeln!(out, "Hello World!"));

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut peripherals = stm32f103xx::Peripherals::take().unwrap();

    //// peripherals.GPIOC.odr.write(|w| unsafe {w.bits(0x44344444)});
    //peripherals.GPIOC.crh.write(|w| w.mode13().output()); // set C13 to outpput
    //peripherals.GPIOC.odr.write(|w| w.odr13().set_bit());

    // enable output C?
    {
        peripherals.RCC.apb2enr.modify(|_, w| w.iopcen().enabled());
        peripherals.RCC.apb2rstr.modify(|_, w| w.iopcrst().set_bit());
        peripherals.RCC.apb2rstr.modify(|_, w| w.iopcrst().clear_bit());
    }



    // split GPIOC into pins
    //apb2.enr().modify(|_, w| w.$iopxenr().enabled());
    //apb2.rstr().modify(|_, w| w.$iopxrst().set_bit());
    //apb2.rstr().modify(|_, w| w.$iopxrst().clear_bit());


    // GPIOC13 into push/pull
    peripherals.GPIOC.crh
        .modify(|r, w| {
            w.cnf13().push();
            w.mode13().output();
            w
        });

     // GPIOC14 into open drain
    peripherals.GPIOC.crh
        .modify(|r, w| {
            w.cnf14().open();
            w.mode14().output();
            w
        });



    let mut flash = peripherals.FLASH.constrain();
    let mut rcc = peripherals.RCC.constrain();
    // let mut led = &mut led as &mut OutputPin;

    let mut speed = 500_u16;

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = stm32f103xx_hal::delay::Delay::new(cp.SYST, clocks);

    {
        let output = OpenDrainOutputWithDelay {
            output: &mut peripherals.GPIOC,
            delay: &mut delay
        };

        let mut wire = OneWire::new(output, false);
        for _ in 0..16 {
            let result = wire.reset();
            if let Ok(ref result) = result {
                if *result {
                    speed = 25_u16;
                }
            }
            stdout(|out| writeln!(out, "reset: {:?}", result));
        }
    }

    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();
    let mut one : PCx<Output<OpenDrain>> = gpioc.pc14.into_open_drain_output(&mut gpioc.crh).downgrade();//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();

    loop {
        led.set_low();
        one.set_low();
        delay.delay_ms(speed);
        /*peripherals.GPIOC.bsrr.write(|w|{
            // set PC13 high
            w.bs13().set();
            // set PC14 high
            w.bs14().set()
        });*/

        led.set_high();
        one.set_high();
        delay.delay_ms(speed);
        /*peripherals.GPIOC.bsrr.write(|w|{
            // set PC13 low
            w.br13().reset();
            // set PC14 low
            w.br14().reset()
        });*/
    }
/*

    let mut state = false;
    loop {
        state = !state;
        if let Ok(ref mut stdout) = stdout {
            writeln!(stdout, "looped, state={}", state);
            writeln!(stdout, "before: {}", peripherals.GPIOC.idr.read().idr13().bit_is_set());
        }
        if state {
            // peripherals.GPIOC.bsrr.write(|w| w.br13().reset());
            peripherals.GPIOC.odr.write(|w| w.odr13().set_bit());
        } else {
            // peripherals.GPIOC.bsrr.write(|w| w.bs13().set());
            peripherals.GPIOC.odr.write(|w| w.odr13().clear_bit());
        }
        for _ in 0..9999 {

        }
        if let Ok(ref mut stdout) = stdout {
            writeln!(stdout, "after:  {}", peripherals.GPIOC.odr.read().odr13().bit_is_set());
            writeln!(stdout, "");
        }
    }*/
}

pub struct OpenDrainOutputWithDelay<'a> {
    output: &'a mut GPIOC,
    delay: &'a mut Delay
}

impl<'a> OpenDrainOutput for OpenDrainOutputWithDelay<'a> {
    fn drain_low(&mut self) {
        self.output.bsrr.write(|w|{
            // set PC14 high
            w.br14().reset()
        });
    }

    fn float_high(&mut self) {
        self.output.bsrr.write(|w|{
            // set PC14 high
            w.bs14().set()
        });
    }

    fn is_high(&self) -> bool {
        self.output.idr.read().idr14().bit_is_set()
    }

    fn delay_us(&mut self, us: u8) {
        self.delay.delay_us(us)
    }
}

#[cfg(debug_assertions)]
fn stdout<A, F: FnOnce(&mut hio::HStdout) -> A>(f: F) {
    static mut STDOUT : Option<hio::HStdout> = None;
    unsafe {
        if STDOUT.is_none() {
            STDOUT = match hio::hstdout() {
                Ok(stdout) => Some(stdout),
                Err(_) => None
            };
        }
        if let Some(ref mut stdout) = STDOUT {
            f(stdout);
        }
    }
}

#[cfg(not(debug_assertions))]
fn stdout<A, F: FnOnce(&mut hio::HStdout) -> A>(f: F) {
    // do nothing
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    // asm::bkpt();
}
