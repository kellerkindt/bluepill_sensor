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

    let mut flash = peripherals.FLASH.constrain();
    let mut rcc = peripherals.RCC.constrain();

    let mut speed = 500_u16;

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = stm32f103xx_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();
    let mut one : PCx<Output<OpenDrain>> = gpioc.pc14.into_open_drain_output(&mut gpioc.crh).downgrade();//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();


    loop {
        led.set_low();
        delay.delay_ms(speed);
        led.set_high();
        delay.delay_ms(speed);

        let mut wire = OneWire::new(&mut one, false);
        for _ in 0..16 {
            let result = wire.reset(&mut delay);
            stdout(|out| writeln!(out, "reset: {:?}", result));
            if let Ok(ref result) = result {
                if *result {
                    speed = 25_u16;
                } else {
                    speed = 500_u16;
                }
            } else {
                speed = 2_000_u16;
            }
        }
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
fn stdout<A, F: FnOnce(&mut hio::HStdout) -> A>(_f: F) {
    // do nothing
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    // asm::bkpt();
}
