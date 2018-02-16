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



use stm32f103xx_hal::prelude::*;

use core::fmt::Write;

use cortex_m::asm;
use self::cortex_m_semihosting::hio;



fn main() {
    stdout(|out| writeln!(out, "Hello World!"));

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut peripherals = stm32f103xx::Peripherals::take().unwrap();

    //// peripherals.GPIOC.odr.write(|w| unsafe {w.bits(0x44344444)});
    //peripherals.GPIOC.crh.write(|w| w.mode13().output()); // set C13 to outpput
    //peripherals.GPIOC.odr.write(|w| w.odr13().set_bit());


    let mut flash = peripherals.FLASH.constrain();
    let mut rcc = peripherals.RCC.constrain();
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);


    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = stm32f103xx_hal::delay::Delay::new(cp.SYST, clocks);

    loop {
        led.set_high();
        delay.delay_ms(1_000_u16);
        led.set_low();
        delay.delay_ms(1_000_u16);
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
