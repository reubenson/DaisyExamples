# ShiftRegisterMidi Library

A persistent canon/cascading MIDI effect with hierarchical gate control for the Daisy platform.

## Overview

The `ShiftRegisterMidi` library implements a latching shift register that creates a persistent canon effect. When you play notes, they cascade through up to 4 voices and remain active until explicitly cleared via Reset. This creates a loop recorder/arpeggiator effect where notes accumulate and cascade through the voices.

## Key Behaviors

### Persistent Canon

Notes are **never removed** from the cascade once played - they remain active even after note-off events. This creates a persistent canon effect where:

- Each new note shifts previous notes down the cascade (voice 0 → voice 1 → voice 2 → voice 3)
- Notes stay in the cascade until `Reset()` is called
- You can build complex cascading patterns by playing notes sequentially or with overlaps

### Hierarchical Gate Control

The library implements a hierarchical gate control system:

#### On Note-On:

- New note is added to the cascade (voice 0)
- **All active voices have their gates re-triggered** (creates unified attack)
- All voices in the cascade sound together with synchronized envelopes

#### On Note-Off:

- **If the newest held note** (voice 0 or most recently added held note) **is released**:
  - **ALL gates release** (all voices go into release phase)
  - Notes remain in cascade with gates closed
- **If an older note** (voices 1-3) **is released**:
  - **Only that voice's gate releases** (only that voice goes into release phase)
  - Other voices remain unaffected
  - Notes remain in cascade

This creates a natural, expressive playing style where:

- The most recent note controls the overall gate state
- Older notes can be released independently
- No spurious retriggering when gates are already closed

## Usage Example

```cpp
#include "midi/ShiftRegisterMidi.h"

// Define MIDI output handler
class DaisyMidiOutput : public envelope_midi::MidiOutput {
  public:
    void Send(const envelope_midi::MidiMessage& message) override {
        daisy::MidiEvent event;
        event.channel = message.channel;

        if (message.type == envelope_midi::MidiMessage::Type::kNoteOn) {
            event.type = daisy::NoteOn;
            event.data[0] = message.note;
            event.data[1] = message.velocity;
        } else {
            event.type = daisy::NoteOff;
            event.data[0] = message.note;
            event.data[1] = 0;
        }

        // Send to hardware MIDI output
        hw.midi.SendMessage(event);
    }
};

// Initialize
DaisyMidiOutput midi_output;
envelope_midi::ShiftRegisterMidi shift_register(&midi_output);

// Handle incoming MIDI
void HandleMidiMessage(daisy::MidiEvent m) {
    if (m.type == daisy::NoteOn) {
        shift_register.HandleNoteOn(m.data[0], m.data[1]);
    } else if (m.type == daisy::NoteOff) {
        shift_register.HandleNoteOff(m.data[0]);
    }
}

// Apply voice states to your synth
void ApplyVoiceStates() {
    const auto& voices = shift_register.GetVoices();

    for (size_t i = 0; i < voices.size(); ++i) {
        if (voices[i].active) {
            if (voices[i].needs_retrigger) {
                envelopes[i].Retrigger(true);
            }
            envelopes[i].gate = voices[i].gate_on;
            synth_voices[i].note = voices[i].note;
            synth_voices[i].velocity = voices[i].velocity;
        } else {
            envelopes[i].gate = false;
        }
    }

    shift_register.ClearRetriggerFlags();
}

// Clear the cascade
void ClearCascade() {
    shift_register.Reset();
}
```

## Playing Examples

### Example 1: Sequential Notes

```
Play C  → Voice 0: C (gate ON)
Play D  → Voice 0: D (gate ON), Voice 1: C (gate ON)
Play E  → Voice 0: E (gate ON), Voice 1: D (gate ON), Voice 2: C (gate ON)
Release E → ALL gates OFF (notes remain in cascade)
Play F  → Voice 0: F (gate ON), Voice 1: E (gate ON), Voice 2: D (gate ON), Voice 3: C (gate ON)
```

### Example 2: Overlapping Notes

```
Play C (hold)     → Voice 0: C (gate ON)
Play D (hold C)   → Voice 0: D (gate ON), Voice 1: C (gate ON)
Release C         → Voice 0: D (gate ON), Voice 1: C (gate OFF)
Release D         → Voice 0: D (gate OFF), Voice 1: C (gate OFF)
```

### Example 3: Complex Cascade

```
Play A (hold)     → Voice 0: A (gate ON)
Play B (hold A)   → Voice 0: B (gate ON), Voice 1: A (gate ON)
Release A         → Voice 0: B (gate ON), Voice 1: A (gate OFF)
Play C (hold B)   → Voice 0: C (gate ON), Voice 1: B (gate ON), Voice 2: A (gate ON)
                    ↑ Note: A's gate re-triggers!
Release B         → Voice 0: C (gate ON), Voice 1: B (gate OFF), Voice 2: A (gate OFF)
Release C         → ALL gates OFF (C was newest held note)
```

## API Reference

### `ShiftRegisterMidi(MidiOutput* output = nullptr)`

Constructor. Optionally takes a MIDI output handler for sending MIDI messages.

### `void SetOutput(MidiOutput* output)`

Set or change the MIDI output handler.

### `void HandleNoteOn(uint8_t note, uint8_t velocity)`

Process a MIDI note-on event. Adds the note to the cascade and re-triggers all active voices.

### `void HandleNoteOff(uint8_t note)`

Process a MIDI note-off event. Releases gates according to hierarchical control rules.

### `void Reset()`

Clear the entire cascade and send note-off messages for all active voices.

### `const std::array<VoiceState, kMaxVoices>& GetVoices() const`

Get the current state of all voices.

### `void ClearRetriggerFlags()`

Clear the `needs_retrigger` flags after they've been processed. Call this after applying voice states.

## VoiceState Structure

```cpp
struct VoiceState {
    bool active;          // Is this voice slot occupied?
    uint8_t note;         // MIDI note number (0-127)
    uint8_t velocity;     // MIDI velocity (0-127)
    bool needs_retrigger; // Should the envelope be retriggered?
    bool gate_on;         // Is the gate currently held (for envelope)?
};
```

## Building and Testing

### Running Unit Tests

The library includes comprehensive unit tests using GoogleTest.

#### First Time Setup

```bash
cd /Users/reubenson/Projects/DaisyExamples/patch/_Envelope/tests
mkdir build
cd build
cmake ..
```

#### Build and Run Tests

```bash
cd /Users/reubenson/Projects/DaisyExamples
cmake --build patch/_Envelope/tests/build
./patch/_Envelope/tests/build/shift_register_tests
```

Or from the tests directory:

```bash
cd /Users/reubenson/Projects/DaisyExamples/patch/_Envelope/tests/build
make
./shift_register_tests
```

#### Test Coverage

The test suite includes:

- Single note behavior
- Multi-note cascading
- Newest note-off releases all gates
- Older note-off releases only that voice
- Overlapping note release behavior
- Sequential note cascade building
- Duplicate note canon effects
- Reset functionality

### Building the Firmware

From the project directory:

```bash
cd /Users/reubenson/Projects/DaisyExamples/patch/_Envelope
make clean && make
```

The compiled binary will be at `build/_Envelope.bin`.

## Design Notes

### Hardware Independence

The library is designed to be hardware-agnostic by using the `MidiOutput` interface. This allows:

- Unit testing without hardware
- Easy integration with different platforms
- Separation of MIDI logic from hardware I/O

### Voice Assignment

Voices are assigned from the end of the queue:

- Voice 0 (newest) = queue index 3
- Voice 1 = queue index 2
- Voice 2 = queue index 1
- Voice 3 (oldest) = queue index 0

This ensures the most recent note always plays on voice 0.

### Retrigger Behavior

The `needs_retrigger` flag is set only when:

1. A voice's state changes (new note assigned, or existing note re-triggered)
2. The gate is currently held (`gate_on = true`)

This prevents spurious envelope retriggering when gates are already closed.

## Future Enhancements

Potential improvements could include:

- Configurable number of voices (currently fixed at 4)
- Different cascade modes (non-persistent, last-note priority, etc.)
- Per-voice gate control configuration
- MIDI note priority modes (high, low, last)

## License

Part of the DaisyExamples project.
