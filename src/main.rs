#![no_std]
#![no_main]
#![deny(intra_doc_link_resolution_failure)]

#[macro_use(entry)]
extern crate cortex_m_rt;

//extern crate panic_abort;
//extern crate panic_halt;
use panic_persist as _;

//#[macro_use(block_while)]
#[macro_use(block)]
extern crate nb;

// mod am2302;
mod ds93c46;
mod io_utils;
mod module;
mod platform;
mod system;
// mod sht1x;
mod cnf;

#[entry]
fn main() -> ! {
    platform::unwrap_builder().run_with_module::<crate::module::ecm::ElectricCounterModule>()
}
