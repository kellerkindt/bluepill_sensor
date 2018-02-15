//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---

#![feature(used)]
#![no_std]


extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;

extern crate stm32f103xx;

use core::fmt::Write;

use cortex_m::asm;
use self::cortex_m_semihosting::hio;



fn main() {
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Hello, world from main!").unwrap();

    let mut peripherals = stm32f103xx::Peripherals::take().unwrap();

    // peripherals.GPIOC.odr.write(|w| unsafe {w.bits(0x44344444)});
    peripherals.GPIOC.crh.write(|w| unsafe {w.bits(0x44344444)}); // set C13 to outpput
    peripherals.GPIOC.odr.write(|w| w.odr13().set_bit());


    let mut state = false;
    loop {
        state = !state;
        writeln!(stdout, "looped, state={}", state);
        writeln!(stdout, "before=0x{:x}", peripherals.GPIOC.odr.read().bits());
        if state {
            peripherals.GPIOC.bsrr.write(|w| unsafe {w.bits(0xFF_FF_FF_FF)});
        } else {
            peripherals.GPIOC.bsrr.write(|w| unsafe {w.bits(0x00_00_00_00)});
        }
        for _ in 0..9999 {

        }
        writeln!(stdout, "after =0x{:x}", peripherals.GPIOC.odr.read().bits());
        writeln!(stdout, "");
    }
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    asm::bkpt();
}



mod stm {

}