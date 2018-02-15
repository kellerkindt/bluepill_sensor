#!/bin/bash

unset CARGO_INCREMENTAL
#XARGO_RUST_SRC=/share/home/michael/.rustup/toolchains/nightly-x86_64-unknown-linux-gnu/lib/rustlib/src \
rustup run nightly xargo build --example hello \
  && sudo openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c targets -c "halt" -c "flash write_image erase target/thumbv7em-none-eabihf/debug/examples/hello" -c "verify_image target/thumbv7em-none-eabihf/debug/examples/hello" -c "reset run"
# -c shutdown
# && sudo openocd -f interface/stlink-v2-1.cfg -f target/stm32f1x.cfg
