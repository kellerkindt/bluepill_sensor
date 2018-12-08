#!/bin/bash

# rustup target add thumbv7m-none-eabi
# rustup run nightly xargo build --release --target=thumbv7m-none-eabi
cargo +nightly build --release
