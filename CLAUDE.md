# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`daisy-embassy` is a Rust embedded async audio framework for the Daisy Seed hardware platform. It provides a high-level interface to Embassy async runtime combined with low-latency audio processing for the STM32H750 microcontroller.

## Supported Hardware

- **Daisy Seed Rev5 (1.1)**: WM8731 codec - default feature
- **Daisy Seed Rev7 (1.2)**: PCM3060 codec - use `--features seed_1_2 --no-default-features`
- **Daisy Patch SM**: PCM3060 codec - use `--features patch_sm --no-default-features`

Only ONE board feature can be active at a time (enforced by compile-time checks in `src/lib.rs`).

## Build and Flash Commands

### Using probe-rs (Debug/Development)

```bash
# Daisy Seed Rev5 (default)
cargo run --example blinky --release

# Daisy Seed Rev7
cargo run --example triangle_wave_tx --features seed_1_2 --no-default-features --release

# Daisy Patch SM
cargo run --example looper --features patch_sm --no-default-features --release
```

### Using DFU (Bootloader Mode)

For Daisy Seed 1.2 without probe-rs:

```bash
./flash_seed_1_2.sh <example_name>
```

This script:
1. Builds with `--features seed_1_2 --no-default-features`
2. Converts ELF to binary using `rust-objcopy`
3. Flashes via `dfu-util` to address 0x08000000
4. Requires board in DFU mode: Hold BOOT, press RESET, release BOOT

## Architecture

### Core Components

- **`new_daisy_board!` macro**: Initializes all peripherals from `embassy_stm32::Peripherals` and creates builder structs
- **`default_rcc()`**: Provides safe clock configuration (480MHz system, 48kHz audio sample rate)
- **Builder pattern**: All peripherals exposed via `XXXBuilder` structs with safe defaults
- **Audio callback**: Runs in high-priority interrupt executor (SAI1, Priority P6) for deterministic audio processing

### Audio Processing Model

Audio runs in a **separate high-priority interrupt executor** isolated from async tasks:

```rust
static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SAI1() {
    AUDIO_EXECUTOR.on_interrupt()
}

// In main:
interrupt::SAI1.set_priority(Priority::P6);
let audio_spawner = AUDIO_EXECUTOR.start(interrupt::SAI1);
audio_spawner.spawn(audio_task(interface));
```

The audio callback processes samples at 48kHz (configurable) with stereo interleaved u32 samples (24-bit audio in 32-bit words).

### Communication Between Audio and Async Tasks

Use atomics with **`Ordering::SeqCst`** to share data between the audio interrupt and async tasks. `Ordering::Relaxed` does NOT guarantee visibility across interrupt boundaries.

```rust
static SHARED_VALUE: AtomicU8 = AtomicU8::new(0);

// In audio callback (interrupt context)
let value = SHARED_VALUE.load(Ordering::SeqCst);

// In async task
SHARED_VALUE.store(42, Ordering::SeqCst);
```

## CRITICAL: USB MIDI Implementation

**USB MIDI packets from `embassy-usb` MidiClass are 4 bytes, NOT 3 bytes.**

Embassy-usb `MidiClass::read_packet()` returns raw USB MIDI packets with the header intact. Parse as **4-byte chunks**:

```rust
for packet in data.chunks(4) {
    if packet.len() >= 4 {
        let _header = packet[0];   // USB MIDI header (cable + CIN)
        let status = packet[1];    // MIDI status byte (0x90 = Note On)
        let data1 = packet[2];     // First data byte (note number)
        let data2 = packet[3];     // Second data byte (velocity)

        // Process MIDI message...
    }
}
```

See `examples/usb_midi_synth.rs` for the correct reference implementation. See `USB_MIDI_SYNTH.md` for detailed explanation.

**DO NOT use `chunks(3)` for USB MIDI parsing** - this causes silent misalignment where packets are parsed incorrectly but no errors are raised.

## Module Structure

- **`src/audio.rs`**: Audio interface, DMA configuration, SAI peripheral setup
- **`src/board.rs`**: `DaisyBoard` struct and `new_daisy_board!` macro
- **`src/codec/`**: Audio codec drivers (WM8731, PCM3060)
- **`src/pins/`**: Pin definitions per board variant
- **`src/flash.rs`**: External flash memory interface
- **`src/sdram.rs`**: SDRAM controller (IS42S16160G on FMC)
- **`src/led.rs`**: User LED control
- **`src/usb.rs`**: USB peripheral configuration

## Sample Rate and Audio Format

- Default sample rate: **48kHz** (configured via codec and SAI)
- Audio format: **24-bit samples in 32-bit words** (u32), stereo interleaved
- Buffer size: Configurable via `AudioPeripherals::set_buffer_size()`
- Conversion helper: `f32_to_u24()` converts float samples to hardware format

## Common Pitfalls

1. **Memory ordering**: Always use `Ordering::SeqCst` for atomics shared between audio interrupt and async tasks
2. **USB MIDI parsing**: Must use 4-byte chunks, not 3-byte
3. **Board features**: Only one board feature can be active; using wrong feature causes hardware mismatch
4. **Audio callback blocking**: Never use async/.await or blocking operations in audio callback - it runs in interrupt context
5. **DMA buffer placement**: Audio buffers must be in appropriate memory regions (handled automatically by framework)

## Testing MIDI

Use the included Python script to send MIDI notes:

```bash
python3 send_midi_note.py 69    # Send note 69 (A4, 440Hz)
python3 send_midi_note.py 48    # Send note 48 (C3)
```

Or connect VMPK (Virtual MIDI Piano Keyboard):

```bash
aconnect -l                      # List MIDI ports
aconnect <vmpk-port>:0 <daisy-port>:0
```

## Resources

- Daisy hardware: https://daisy.audio/hardware/
- Embassy framework: https://github.com/embassy-rs/embassy
- probe-rs: https://probe.rs/
- STM32H750 reference: STM32H7 series datasheet
