#!/bin/bash

unset CARGO_INCREMENTAL
sudo openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
