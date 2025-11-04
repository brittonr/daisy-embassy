#!/bin/sh
# Demo script for multi-source granular synth
# Plays notes on all 4 MIDI channels to showcase different sources

echo "Multi-Source Granular Synthesizer Demo"
echo "======================================"
echo ""
echo "Playing different sources..."
echo ""

echo "Channel 1 (Sine Sweep):"
python3 send_midi_channel.py 60 1 &
sleep 0.3
python3 send_midi_channel.py 64 1 &
sleep 0.3
python3 send_midi_channel.py 67 1 &
sleep 1.0

echo ""
echo "Channel 2 (Square Wave):"
python3 send_midi_channel.py 60 2 &
sleep 0.3
python3 send_midi_channel.py 64 2 &
sleep 0.3
python3 send_midi_channel.py 67 2 &
sleep 1.0

echo ""
echo "Channel 3 (Sawtooth):"
python3 send_midi_channel.py 60 3 &
sleep 0.3
python3 send_midi_channel.py 64 3 &
sleep 0.3
python3 send_midi_channel.py 67 3 &
sleep 1.0

echo ""
echo "Channel 4 (Noise Burst):"
python3 send_midi_channel.py 60 4 &
sleep 0.3
python3 send_midi_channel.py 64 4 &
sleep 0.3
python3 send_midi_channel.py 67 4 &
sleep 1.0

echo ""
echo "Now all channels together (granular cloud!):"
for note in 60 62 64 65 67 69 71 72; do
    python3 send_midi_channel.py $note 1 &
    python3 send_midi_channel.py $note 2 &
    python3 send_midi_channel.py $note 3 &
    python3 send_midi_channel.py $note 4 &
    sleep 0.1
done

wait
echo ""
echo "Demo complete!"
