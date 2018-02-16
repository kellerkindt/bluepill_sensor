#!/bin/bash

unset CARGO_INCREMENTAL

rustup run nightly xargo build --release \
 && sudo openocd \
	-f interface/stlink-v2.cfg \
	-f target/stm32f1x.cfg \
	-c init \
	-c targets \
	-c "halt" \
	-c "flash write_image erase target/thumbv7em-none-eabihf/release/hebeanlage" \
	-c "verify_image target/thumbv7em-none-eabihf/release/hebeanlage" \
	-c "reset run" \
	-c shutdown
