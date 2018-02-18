//! Prints "Hello, world!" on the OpenOCD console using semihosting
//!
//! ---
#![feature(core_intrinsics)]
#![feature(lang_items)]
#![feature(used)]
#![no_std]


extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;

extern crate stm32f103xx;
extern crate stm32f103xx_hal;

extern crate onewire;
extern crate pcd8544;

use stm32f103xx_hal::prelude::*;
use stm32f103xx_hal::prelude::_embedded_hal_digital_OutputPin as OutputPin;
use stm32f103xx_hal::gpio::Output;
use stm32f103xx_hal::gpio::OpenDrain;
use stm32f103xx_hal::gpio::gpioc::PCx;
use stm32f103xx_hal::delay::Delay;

use stm32f103xx::GPIOC;

use core::fmt::Write;
use core::result::*;

use cortex_m::asm;
use self::cortex_m_semihosting::hio;

use onewire::*;
use pcd8544::*;



#[cfg(debug_assertions)]
fn stdout<A, F: FnOnce(&mut hio::HStdout) -> A>(f: F) {
    /*
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
    */
}

#[cfg(not(debug_assertions))]
fn stdout<A, F: FnOnce(&mut hio::HStdout) -> A>(_f: F) {
    // do nothing
}

fn main() {
    let mut test = 0_u8;
    lolz(&mut test as *mut _);
}

fn lolz(probe0: *mut u8) {
    let mut probe1 = 0u8;

    stdout(|out| writeln!(out, "Hello World!"));

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut peripherals = stm32f103xx::Peripherals::take().unwrap();

    let mut flash = peripherals.FLASH.constrain();
    let mut rcc = peripherals.RCC.constrain();

    let mut speed = 500_u16;

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = stm32f103xx_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = peripherals.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();
    let mut one : PCx<Output<OpenDrain>> = gpioc.pc14.into_open_drain_output(&mut gpioc.crh).downgrade();//.into_push_pull_output(&mut gpioc.crh);//.into_floating_input().is_high();


    let mut pcd_gnd   = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_light = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_vcc   = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_clk   = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
    let mut pcd_din   = gpioa.pa8 .into_push_pull_output(&mut gpioa.crh);
    let mut pcd_dc    = gpioa.pa9 .into_push_pull_output(&mut gpioa.crh);
    let mut pcd_ce    = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);
    let mut pcd_rst   = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);


    pcd_gnd  .set_low();
    pcd_light.set_high();
    pcd_vcc  .set_high();

    let mut display = PCD8544::new(
        &mut pcd_clk,
        &mut pcd_din,
        &mut pcd_dc,
        &mut pcd_ce,
        &mut pcd_rst,
        &mut pcd_light,
    );

    display.reset();
    display.write_str("Hello World");


    /*
    let mut test = 0u8;
    let mut probe2 = [0u8; 18024];
    let mut probe3 = 0u8;
    display.clear();
    //write!(display, "sizeof: {}", core::mem::size_of::<PCx<Output<OpenDrain>>>());
    write!(display, "address: {}  {}  {}", probe0 as usize - 0x20000000_usize, (&mut "tes" as *mut _) as usize - 0x20000000_usize, (&mut probe2[0] as *mut _) as usize - 0x20000000_usize);
    */
    display.clear();
    write!(display, "address: {}  {}", probe0 as usize - 0x20000000_usize, (&mut probe1 as *mut _) as usize - 0x20000000_usize);

    loop {
        led.set_low();
        delay.delay_ms(speed);
        led.set_high();
        delay.delay_ms(speed);

        let mut wire = OneWire::new(&mut one, false);
        for _ in 0..16 {
            let result = wire.reset(&mut delay);
            stdout(|out| writeln!(out, "reset: {:?}", result));
            // display.clear();
            // write!(display, "{:?}", result);
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
        display.set_light(true);
        display.clear();
        writeln!(display, "Search: ");
        let mut search = OneWireDeviceSearch::new();
        match wire.search_next(&mut search, &mut delay) {
            Err(e) => {
                write!(display, "{:?}", e);
            },
            Ok(addr) => {
                if let Some(addr) = addr {
                    write!(display, "# {}", addr);
                } else {
                    write!(display, "None");
                }
            }
        }
    }
}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    // asm::bkpt();
}