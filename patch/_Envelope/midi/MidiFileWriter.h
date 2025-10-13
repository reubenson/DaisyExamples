#pragma once

#include "fatfs.h"
#include <stdint.h>

namespace daisy
{

/** @brief Simple MIDI File Writer for Standard MIDI File (SMF) Format 0
 *  
 *  This class provides basic functionality to write MIDI files to an SD card.
 *  Currently supports Format 0 (single track) MIDI files.
 */
class MidiFileWriter
{
  public:
    MidiFileWriter() {}
    ~MidiFileWriter() {}

    /** @brief Generate a test MIDI file containing a C major scale (up and down)
     *  
     *  @param filename The name of the file to create (e.g., "test_scale.mid")
     *  @return true if file was created successfully, false otherwise
     */
    static bool GenerateCMajorScaleFile(const char* filename);

  private:
    /** @brief Write 32-bit big-endian integer to buffer */
    static void WriteBigEndian32(uint8_t* buffer, uint32_t value);
    
    /** @brief Write 16-bit big-endian integer to buffer */
    static void WriteBigEndian16(uint8_t* buffer, uint16_t value);
    
    /** @brief Encode delta time as variable-length quantity
     *  
     *  @param buffer Output buffer
     *  @param value Delta time value
     *  @return Number of bytes written
     */
    static uint32_t WriteVariableLength(uint8_t* buffer, uint32_t value);
    
    /** @brief Write MIDI header chunk (MThd)
     *  
     *  @param file File handle
     *  @param format MIDI file format (0, 1, or 2)
     *  @param tracks Number of tracks
     *  @param division Ticks per quarter note
     *  @return true on success
     */
    static bool WriteHeader(FIL* file, uint16_t format, uint16_t tracks, uint16_t division);
    
    /** @brief Write a MIDI note on event to buffer
     *  
     *  @param buffer Output buffer
     *  @param deltaTime Time since last event
     *  @param channel MIDI channel (0-15)
     *  @param note Note number (0-127)
     *  @param velocity Velocity (0-127)
     *  @return Number of bytes written
     */
    static uint32_t WriteMidiNoteOn(uint8_t* buffer, uint32_t deltaTime, uint8_t channel, uint8_t note, uint8_t velocity);
    
    /** @brief Write a MIDI note off event to buffer
     *  
     *  @param buffer Output buffer
     *  @param deltaTime Time since last event
     *  @param channel MIDI channel (0-15)
     *  @param note Note number (0-127)
     *  @param velocity Release velocity (0-127)
     *  @return Number of bytes written
     */
    static uint32_t WriteMidiNoteOff(uint8_t* buffer, uint32_t deltaTime, uint8_t channel, uint8_t note, uint8_t velocity);
    
    /** @brief Write tempo meta event to buffer
     *  
     *  @param buffer Output buffer
     *  @param deltaTime Time since last event
     *  @param microsecondsPerQuarter Tempo in microseconds per quarter note
     *  @return Number of bytes written
     */
    static uint32_t WriteTempoEvent(uint8_t* buffer, uint32_t deltaTime, uint32_t microsecondsPerQuarter);
    
    /** @brief Write end of track meta event to buffer
     *  
     *  @param buffer Output buffer
     *  @param deltaTime Time since last event
     *  @return Number of bytes written
     */
    static uint32_t WriteEndOfTrack(uint8_t* buffer, uint32_t deltaTime);
};

} // namespace daisy

