//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---

#![feature(used)]
#![no_std]

extern crate stm32f103xx;


extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;


use core::fmt::Write;

use cortex_m::asm;
use cortex_m_semihosting::hio;



fn main() {
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Hello, world!").unwrap();

    let mut peripherals = stm32f103xx::Peripherals::take().unwrap();
    let usb = peripherals.USB;
    drop(usb);
    peripherals.GPIOB.odr.write(|w| unsafe {w.bits(1)});

//    peripherals.GPIOE.moder.modify(|_, w| w.moder9().output());

    // peripherals.GPIOC.crl.write(|w| unsafe {w.bits(13)});
    peripherals.GPIOC.crh.write(|w| unsafe {w.bits(0x44344444)}); // set C13 to outpput
    peripherals.GPIOC.odr.write(|w| unsafe {w.odr13().set_bit()});


    let mut state = false;
    loop {
        for _ in 0..9999 {

        }
        state = !state;
        writeln!(stdout, "looped, state={}", state);
        writeln!(stdout, "before=0x{:x}", peripherals.GPIOC.odr.read().bits());
        if state {
            // peripherals.GPIOC.bsrr.write(|w| w.bs13().set());
            peripherals.GPIOC.odr.write(|w| unsafe {w.bits(0xFFFF_FFFFu32)});
        } else {
            // peripherals.GPIOC.bsrr.write(|w| w.br13().bit(true));
            peripherals.GPIOC.odr.write(|w| unsafe {w.odr13().clear_bit()});
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