#pragma once

#include "fatfs.h"
#include <stdint.h>
#include <vector>

namespace daisy
{

// Forward declaration
struct MidiEvent;

/** @brief MIDI File Reader for Standard MIDI File (SMF) Format 0 and 1
 *  
 *  This class provides functionality to read and parse MIDI files from an SD card.
 *  Supports Format 0 (single track) and Format 1 (multiple tracks merged).
 */
class MidiFileReader
{
  public:
    MidiFileReader() : fileLoaded_(false), currentEventIndex_(0), totalTicks_(0), ticksPerQuarterNote_(480) {}
    ~MidiFileReader() {}

    /** @brief Load and parse a MIDI file from SD card
     *  
     *  @param filename The name of the file to load (e.g., "song.mid")
     *  @return true if file was loaded and parsed successfully, false otherwise
     */
    bool LoadFile(const char* filename);

    /** @brief Get the next MIDI event to play at the given tick
     *  
     *  @param currentTick Current playback position in ticks
     *  @param event Output event structure
     *  @return true if an event was found, false if no more events at this tick
     */
    bool GetNextEvent(uint32_t currentTick, MidiEvent* event);

    /** @brief Reset playback to the beginning of the file
     */
    void Reset();

    /** @brief Get the total length of the file in ticks
     *  
     *  @return Total ticks in the file
     */
    uint32_t GetTotalTicks() const { return totalTicks_; }

    /** @brief Get the timing resolution (ticks per quarter note)
     *  
     *  @return Ticks per quarter note
     */
    uint16_t GetTicksPerQuarterNote() const { return ticksPerQuarterNote_; }

    /** @brief Check if a file is currently loaded
     *  
     *  @return true if file is loaded, false otherwise
     */
    bool IsFileLoaded() const { return fileLoaded_; }

  private:
    /** @brief MIDI event structure for playback */
    struct MidiEventData
    {
        uint32_t tick;          // Absolute tick time
        uint8_t status;         // MIDI status byte
        uint8_t data1;          // First data byte (note, controller, etc.)
        uint8_t data2;          // Second data byte (velocity, value, etc.)
        uint8_t channel;        // MIDI channel (0-15)
        bool isNoteOn;          // true for note-on, false for note-off
    };

    /** @brief Read 32-bit big-endian integer from buffer */
    uint32_t ReadBigEndian32(const uint8_t* buffer);
    
    /** @brief Read 16-bit big-endian integer from buffer */
    uint16_t ReadBigEndian16(const uint8_t* buffer);
    
    /** @brief Decode variable-length quantity from buffer
     *  
     *  @param buffer Input buffer
     *  @param bytesRead Output parameter for number of bytes consumed
     *  @return Decoded value
     */
    uint32_t ReadVariableLength(const uint8_t* buffer, uint32_t* bytesRead);
    
    /** @brief Parse MIDI file header chunk (MThd)
     *  
     *  @param buffer Buffer containing header data
     *  @return true on success
     */
    bool ParseHeader(const uint8_t* buffer);
    
    /** @brief Parse MIDI track chunk (MTrk)
     *  
     *  @param buffer Buffer containing track data
     *  @param length Length of track data
     *  @return true on success
     */
    bool ParseTrack(const uint8_t* buffer, uint32_t length);

    bool fileLoaded_;                           // Is a file currently loaded?
    std::vector<MidiEventData> events_;         // Sorted list of MIDI events
    uint32_t currentEventIndex_;                // Current position in events list
    uint32_t totalTicks_;                       // Total length in ticks
    uint16_t ticksPerQuarterNote_;             // Timing resolution
};

} // namespace daisy
