#!/bin/bash

unset CARGO_INCREMENTAL
BIN="target/thumbv7m-none-eabi/release/hebeanlage"

rustup run nightly xargo build --release \
 && sudo openocd \
	-f interface/stlink-v2.cfg \
	-f target/stm32f1x.cfg \
	-c init \
	-c targets \
	-c "halt" \
	-c "flash write_image erase $BIN" \
	-c "verify_image $BIN" \
	-c "reset run" \
	-c shutdown
