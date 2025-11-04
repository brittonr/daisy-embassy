#!/usr/bin/env python3
"""Send MIDI note to Daisy board using aplaymidi"""
import subprocess
import sys
import struct

def send_note(note=69, duration=0.5):
    """Create and send a MIDI file with one note"""

    # MIDI file header
    header = b'MThd'
    header += struct.pack('>I', 6)  # Header length
    header += struct.pack('>HHH', 0, 1, 480)  # Format 0, 1 track, 480 ticks/quarter

    # Track chunk
    track = b'MTrk'
    events = bytearray()

    # Note On at time 0
    events.extend([0x00, 0x90, note, 0x64])  # Delta 0, Note On, note, velocity

    # Note Off after duration
    delay_ticks = int(480 * duration)
    if delay_ticks < 128:
        events.append(delay_ticks)
    else:
        events.extend([0x81, delay_ticks & 0x7F])

    events.extend([0x80, note, 0x00])  # Note Off

    # End of track
    events.extend([0x00, 0xFF, 0x2F, 0x00])

    track += struct.pack('>I', len(events))
    track += events

    # Write MIDI file
    midi_file = '/tmp/daisy_note.mid'
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

    print(f"Sending note {note} to port {port}:0...")
    result = subprocess.run(['aplaymidi', '-p', f'{port}:0', midi_file],
                          capture_output=True, text=True)

    if result.returncode == 0:
        print(f"Success! Note {note} sent.")
        return True
    else:
        print(f"Error: {result.stderr}")
        return False

if __name__ == '__main__':
    note = int(sys.argv[1]) if len(sys.argv) > 1 else 69
    send_note(note)
