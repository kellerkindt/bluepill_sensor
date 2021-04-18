#!/bin/bash

unset CARGO_INCREMENTAL
BIN="${CARGO_TARGET_DIR:-../target}/thumbv7m-none-eabi/release/bluepill_sensor"
echo $BIN

# china clone CPU-ID \
#        -c "set CPUTAPID 0x2ba01477" \

./build.sh "$@" \
 && openocd \
	-f interface/stlink-v2.cfg \
	-f target/stm32f1x.cfg \
	-c init \
	-c targets \
        -c "halt" \
	-c "flash write_image erase $BIN" \
	-c "verify_image $BIN" \
	-c "reset run" \
        -c shutdown
