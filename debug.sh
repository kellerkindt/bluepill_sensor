#!/bin/bash

unset CARGO_INCREMENTAL
#XARGO_RUST_SRC=/share/home/michael/.rustup/toolchains/nightly-x86_64-unknown-linux-gnu/lib/rustlib/src \
rustup run nightly xargo build \
 && arm-none-eabi-gdb target/thumbv7em-none-eabihf/debug/hebeanlage
