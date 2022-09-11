#![no_std]
#![no_main]
#![deny(broken_intra_doc_links)]

#[macro_use(entry)]
extern crate cortex_m_rt;

//extern crate panic_abort;
//extern crate panic_halt;
use panic_persist as _;

//#[macro_use(block_while)]
#[macro_use(block)]
extern crate nb;

#[macro_use]
extern crate sensor_common;

#[macro_use]
mod nb_ext;

#[macro_use]
mod props;

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
    platform::unwrap_builder().run_with_module::<crate::module::FeaturedModule>()
}
