#!/usr/bin/env python3
"""Send MIDI note on specific channel to Daisy board"""
import subprocess
import sys
import struct

def send_note(note=60, channel=1, duration=0.5):
    """Create and send a MIDI file with one note on specified channel"""

    # MIDI channels are 0-15, user provides 1-16
    channel = max(1, min(16, channel)) - 1

    # MIDI file header
    header = b'MThd'
    header += struct.pack('>I', 6)
    header += struct.pack('>HHH', 0, 1, 480)

    # Track chunk
    track = b'MTrk'
    events = bytearray()

    # Note On (0x90 + channel)
    events.extend([0x00, 0x90 + channel, note, 0x64])

    # Note Off after duration
    delay_ticks = int(480 * duration)
    if delay_ticks < 128:
        events.append(delay_ticks)
    else:
        events.extend([0x81, delay_ticks & 0x7F])

    events.extend([0x80 + channel, note, 0x00])

    # End of track
    events.extend([0x00, 0xFF, 0x2F, 0x00])

    track += struct.pack('>I', len(events))
    track += events

    # Write MIDI file
    midi_file = '/tmp/daisy_note_ch.mid'
    with open(midi_file, 'wb') as f:
        f.write(header + track)

    # Find Daisy port
    result = subprocess.run(['aconnect', '-l'], capture_output=True, text=True)
    port = None
    for line in result.stdout.split('\n'):
        if 'USB MIDI' in line or 'Synth' in line or 'Daisy' in line or 'Granular' in line:
            # Look for "client XX:" pattern
            if line.strip().startswith('client'):
                parts = line.strip().split()
                if len(parts) > 1:
                    # parts[1] is "20:" - extract the number
                    port = parts[1].replace(':', '')
                    break

    if not port:
        print("Error: Could not find Daisy MIDI device")
        print("Available devices:")
        print(result.stdout)
        return False

    print(f"Sending note {note} on channel {channel + 1} to port {port}:0...")
    result = subprocess.run(['aplaymidi', '-p', f'{port}:0', midi_file],
                          capture_output=True, text=True)

    if result.returncode == 0:
        print(f"Success! Note {note} on channel {channel + 1}")
        return True
    else:
        print(f"Error: {result.stderr}")
        return False

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 send_midi_channel.py <note> [channel] [duration]")
        print("  note: MIDI note number (0-127)")
        print("  channel: MIDI channel (1-16, default 1)")
        print("  duration: note duration in seconds (default 0.5)")
        print("")
        print("Examples:")
        print("  python3 send_midi_channel.py 60 1    # Note 60 on channel 1 (Sine)")
        print("  python3 send_midi_channel.py 64 2    # Note 64 on channel 2 (Square)")
        print("  python3 send_midi_channel.py 67 3    # Note 67 on channel 3 (Saw)")
        print("  python3 send_midi_channel.py 72 4    # Note 72 on channel 4 (Noise)")
        sys.exit(1)

    note = int(sys.argv[1])
    channel = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5

    send_note(note, channel, duration)
