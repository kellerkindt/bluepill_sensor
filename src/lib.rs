#![no_std]
#![deny(broken_intra_doc_links)]

#[cfg(not(feature = "board-rev-specified"))]
compile_error!("Missing board revision selection (board-rev1, board-rev-2, board-rev-3, ...)");

//#[macro_use(block_while)]
#[macro_use(block)]
extern crate nb;

#[macro_use]
pub mod nb_ext;

#[macro_use]
pub mod props;

// pub mod am2302;
pub mod ds93c46;
pub mod io_utils;
pub mod module;
pub mod platform;
pub mod system;
// pub mod sht1x;
pub mod cnf;
