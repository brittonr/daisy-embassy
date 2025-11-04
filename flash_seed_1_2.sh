#!/bin/sh

if [ -z "$1" ]; then
    echo "Usage: $0 <example_name>"
    echo "Example: $0 simple_tone"
    exit 1
fi

EXAMPLE_NAME="$1"

echo "Building example: $EXAMPLE_NAME for Daisy Seed 1.2"
cargo build --release --example "$EXAMPLE_NAME" --no-default-features --features seed_1_2

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "Converting to binary..."
rust-objcopy -O binary "target/thumbv7em-none-eabihf/release/examples/$EXAMPLE_NAME" "target/thumbv7em-none-eabihf/release/examples/$EXAMPLE_NAME.bin"

if [ $? -ne 0 ]; then
    echo "Binary conversion failed!"
    exit 1
fi

echo "Flashing via DFU..."
dfu-util -a 0 -s 0x08000000 -D "target/thumbv7em-none-eabihf/release/examples/$EXAMPLE_NAME.bin"

if [ $? -eq 0 ]; then
    echo "Successfully flashed $EXAMPLE_NAME to Daisy Seed 1.2!"
    echo "Press RESET button to run the program."
else
    echo "Flash failed!"
    exit 1
fi