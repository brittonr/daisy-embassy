# USB MIDI Synthesizer - THE BIBLE

## The Critical Discovery

**USB MIDI packets are 4 bytes, not 3!**

All examples in the wild parse as `chunks(3)`, which is WRONG for USB MIDI. Embassy-usb `MidiClass::read_packet()` does NOT strip the USB MIDI header.

### USB MIDI Packet Format

```
[Header Byte] [Status Byte] [Data Byte 1] [Data Byte 2]
     0x09         0x90          0x45          0x64
   Cable+CIN    Note On       Note 69       Vel 100
```

## The Working Example

**File:** `examples/usb_midi_synth.rs`

This is the CORRECT implementation. Key points:

### Correct Parsing (Line 90)

```rust
// CRITICAL: Parse as 4-byte chunks!
for packet in data.chunks(4) {
    if packet.len() >= 4 {
        let _header = packet[0];  // USB MIDI header
        let status = packet[1];   // MIDI status byte
        let note = packet[2];     // Note number
        let velocity = packet[3]; // Velocity

        // Process the MIDI message...
    }
}
```

### Memory Ordering (Lines 44-45, 104-105)

Use `Ordering::SeqCst` for atomics shared between async MIDI handler and interrupt audio callback:

```rust
// In audio callback (high-priority interrupt)
let note = CURRENT_NOTE.load(Ordering::SeqCst);

// In MIDI handler (async task)
CURRENT_NOTE.store(note, Ordering::SeqCst);
```

`Ordering::Relaxed` does NOT guarantee visibility across interrupt boundaries!

## How to Use

### Flash the firmware

```sh
./flash_seed_1_2.sh usb_midi_synth
```

### Send MIDI notes

```sh
python3 send_midi_note.py 60   # Middle C
python3 send_midi_note.py 69   # A4 (440Hz)
python3 send_midi_note.py 48   # Low C
```

### Use with VMPK

```sh
# Find ports
aconnect -l

# Connect VMPK to Daisy
aconnect <vmpk-port>:0 <daisy-port>:0
```

## Why This Matters

Every single MIDI example in the daisy-embassy repo uses `chunks(3)`, which silently fails. The packets arrive, but parsing is misaligned:

**Wrong (chunks 3):**
```
Chunk 1: [0x09, 0x90, 0x45]  <- Header interpreted as status!
Chunk 2: [0x64, 0x08, 0x80]  <- Velocity + next packet mixed
```

**Correct (chunks 4):**
```
Packet 1: [0x09, 0x90, 0x45, 0x64]  <- Complete Note On
Packet 2: [0x08, 0x80, 0x45, 0x00]  <- Complete Note Off
```

## Reference

- USB MIDI spec: 4-byte packets (1 header + 3 MIDI bytes)
- Embassy-usb MidiClass: Returns raw USB packets
- Header byte: Upper nibble = cable, lower nibble = Code Index Number
- Status byte: MIDI message type (0x90 = Note On, 0x80 = Note Off)

**THIS IS THE WAY.**
