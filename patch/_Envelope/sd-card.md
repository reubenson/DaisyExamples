# MIDI File Playback Debugging Notes

## Project Overview

**Goal**: Implement MIDI file playback from SD card on Daisy Patch, triggered by knob 4 on Storage panel (0.5-1.0 = play, 0-0.5 = stop).

## Current Status: **BLOCKED** ❌

The system is stuck at a fundamental level - MIDI notes are not being generated despite timing system working correctly.

## What Works ✅

1. **SD Card Detection**: SD card is mounted and files are detected (6 MIDI files found)
2. **UI System**: Storage panel displays correctly, knob 4 controls playback state
3. **Timing System**: `elapsedMicroseconds` and `ticksElapsed` are incrementing correctly
4. **Playback State**: System correctly enters "Playing" mode when knob 4 > 0.5
5. **File Operations**: Basic SD card operations work (file listing, mounting)

## What Doesn't Work ❌

1. **MIDI Note Generation**: No MIDI notes are being generated or heard
2. **Debug Messages**: Most debug messages are not appearing on screen
3. **File Reading**: SD card file reading hangs (even 1 byte read hangs)

## Debugging Journey

### Phase 1: Initial Implementation

- **Created**: `MidiFileReader` class for parsing MIDI files
- **Added**: MIDI playback state management in main application
- **Result**: Application crashed when turning knob 4

### Phase 2: Crash Debugging

- **Problem**: `f_open()` crashed in knob handler context
- **Solution**: Moved file operations to main loop context
- **Result**: Crash resolved, but playback not working

### Phase 3: Playback Debugging

- **Problem**: MIDI events sent via `hw.midi.SendMessage()` not heard
- **Solution**: Route events through `HandleMidiMessage()` for voice allocation
- **Result**: Still no MIDI output

### Phase 4: File Loading Issues

- **Problem**: `MidiFileReader::LoadFile()` failing silently
- **Attempted**: Simplified file reading test (14-byte header read)
- **Result**: `f_read()` hanging even for minimal reads

### Phase 5: SD Card Bypass

- **Decision**: Bypass SD card entirely, use programmatic test sequence
- **Implementation**: Generate Middle C (note 60) every 480 ticks
- **Result**: Timing works, but no MIDI notes generated

### Phase 6: Deep Debugging

- **Added**: Extensive debug output to track execution flow
- **Current State**:
  - `elapsedMicroseconds` and `ticksElapsed` incrementing ✅
  - `midiPlaybackTick` advancing ✅
  - Debug messages "Elapsed:X Ticks:Y" appearing ✅
  - **Missing**: "TickCheck:", "InNoteSection:", "NoteOn:" messages ❌

## Current Code State

### Key Variables

```cpp
bool midiPlaybackEnabled = false;        // Playback state
uint32_t midiPlaybackTick = 0;           // Current playback position
uint32_t lastMidiEventTime = 0;          // Last timing update
uint32_t midiTickInterval = 0;           // Microseconds per tick
bool midiFileLoaded = false;             // File loading state
char lastMidiEvent[32] = "None";         // Debug display
uint32_t lastMidiEventTick = 0;         // Debug display
```

### Test Sequence Logic

```cpp
// Every 480 ticks (quarter note at 120 BPM):
if(midiPlaybackTick - lastNoteTick >= 480)
{
    // Generate NoteOn/NoteOff for Middle C (note 60)
    // Route through HandleMidiMessage()
}
```

## Debugging Attempts Made

### 1. Context Testing

- **Test**: Move file operations from knob handler to main loop
- **Result**: Resolved crash, but playback still not working

### 2. MIDI Routing Testing

- **Test**: Route MIDI events through `HandleMidiMessage()` instead of direct send
- **Result**: No change in output

### 3. File Reading Testing

- **Test**: Simplified file reading (1 byte, 14 bytes, full header)
- **Result**: All file reads hang, even minimal operations

### 4. Programmatic Sequence Testing

- **Test**: Generate MIDI events programmatically (bypass file reading)
- **Result**: Timing works, but no MIDI events generated

### 5. Debug Output Testing

- **Test**: Add extensive debug messages to track execution
- **Result**: Only basic timing messages appear, note generation messages missing

## Current Debugging Strategy

### What We Know Works

1. **Timing Calculation**: `elapsedMicroseconds / midiTickInterval` produces correct `ticksElapsed`
2. **Tick Advancement**: `midiPlaybackTick` increments correctly
3. **UI Updates**: Debug messages appear on OLED screen

### What We're Missing

1. **TickCheck Messages**: Should show `ticksElapsed` value every 60 ticks
2. **InNoteSection Messages**: Should show when note generation code is reached
3. **NoteOn/NoteOff Messages**: Should show when MIDI events are generated

## Potential Issues

### 1. Static Variable Scope

- **Problem**: Static variables in `UpdateMidiPlayback()` might not be resetting correctly
- **Evidence**: Debug counters might not be advancing as expected

### 2. Timing Precision

- **Problem**: `midiTickInterval` calculation might be wrong
- **Current**: `60000000 / 120 / 480 = 1041.67` microseconds per tick
- **Issue**: Integer division might cause timing drift

### 3. Conditional Logic

- **Problem**: The `if(ticksElapsed > 0)` condition might never be true
- **Evidence**: "TickCheck:" messages not appearing suggests this condition fails

### 4. Memory/Stack Issues

- **Problem**: Stack overflow or memory corruption affecting execution
- **Evidence**: Debug messages disappearing suggests execution flow issues

## Next Steps (If Continuing)

### Immediate Actions

1. **Verify TickCheck Messages**: Ensure `ticksElapsed > 0` condition is being met
2. **Check Static Variables**: Verify debug counters are advancing correctly
3. **Simplify Logic**: Remove complex conditions, use direct tick-based triggers

### Alternative Approaches

1. **Hardware MIDI Test**: Send MIDI directly via `hw.midi.SendMessage()` to verify hardware
2. **Voice Allocation Test**: Test `HandleMidiMessage()` with manual MIDI events
3. **Timing Simplification**: Use fixed intervals instead of calculated timing

## Files Modified

- `patch/_Envelope/_Envelope.cpp` - Main application with MIDI playback logic
- `patch/_Envelope/midi/MidiFileReader.h` - MIDI file parsing class header
- `patch/_Envelope/midi/MidiFileReader.cpp` - MIDI file parsing implementation
- `patch/_Envelope/Makefile` - Build configuration

## Conclusion

The MIDI file playback system is fundamentally blocked at the note generation level. Despite a working timing system and correct playback state management, MIDI events are not being generated or processed. The issue appears to be in the execution flow of the `UpdateMidiPlayback()` function, specifically around the conditional logic for tick processing and note generation.

**Recommendation**: Consider starting with a simpler approach - direct MIDI output testing to verify the hardware and voice allocation system work correctly, then build up the timing and file reading functionality incrementally.
