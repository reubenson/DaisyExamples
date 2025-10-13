#include "MidiFileWriter.h"
#include <string.h>

namespace daisy
{

void MidiFileWriter::WriteBigEndian32(uint8_t* buffer, uint32_t value)
{
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

void MidiFileWriter::WriteBigEndian16(uint8_t* buffer, uint16_t value)
{
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

uint32_t MidiFileWriter::WriteVariableLength(uint8_t* buffer, uint32_t value)
{
    uint32_t bytes_written = 0;
    uint8_t temp[4];
    uint32_t temp_len = 0;
    
    // Build the variable-length quantity in reverse
    temp[temp_len++] = value & 0x7F;
    value >>= 7;
    
    while(value > 0)
    {
        temp[temp_len++] = (value & 0x7F) | 0x80;
        value >>= 7;
    }
    
    // Write in correct order (reverse of how we built it)
    for(int i = temp_len - 1; i >= 0; i--)
    {
        buffer[bytes_written++] = temp[i];
    }
    
    return bytes_written;
}

bool MidiFileWriter::WriteHeader(FIL* file, uint16_t format, uint16_t tracks, uint16_t division)
{
    uint8_t header[14];
    UINT bytes_written;
    
    // MThd chunk
    header[0] = 'M';
    header[1] = 'T';
    header[2] = 'h';
    header[3] = 'd';
    
    // Chunk length (always 6 for header)
    WriteBigEndian32(&header[4], 6);
    
    // Format type
    WriteBigEndian16(&header[8], format);
    
    // Number of tracks
    WriteBigEndian16(&header[10], tracks);
    
    // Ticks per quarter note (division)
    WriteBigEndian16(&header[12], division);
    
    FRESULT res = f_write(file, header, 14, &bytes_written);
    return (res == FR_OK && bytes_written == 14);
}

uint32_t MidiFileWriter::WriteMidiNoteOn(uint8_t* buffer, uint32_t deltaTime, uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint32_t bytes_written = 0;
    
    // Write delta time
    bytes_written += WriteVariableLength(&buffer[bytes_written], deltaTime);
    
    // Write note on message (0x90 + channel)
    buffer[bytes_written++] = 0x90 | (channel & 0x0F);
    buffer[bytes_written++] = note & 0x7F;
    buffer[bytes_written++] = velocity & 0x7F;
    
    return bytes_written;
}

uint32_t MidiFileWriter::WriteMidiNoteOff(uint8_t* buffer, uint32_t deltaTime, uint8_t channel, uint8_t note, uint8_t velocity)
{
    uint32_t bytes_written = 0;
    
    // Write delta time
    bytes_written += WriteVariableLength(&buffer[bytes_written], deltaTime);
    
    // Write note off message (0x80 + channel)
    buffer[bytes_written++] = 0x80 | (channel & 0x0F);
    buffer[bytes_written++] = note & 0x7F;
    buffer[bytes_written++] = velocity & 0x7F;
    
    return bytes_written;
}

uint32_t MidiFileWriter::WriteTempoEvent(uint8_t* buffer, uint32_t deltaTime, uint32_t microsecondsPerQuarter)
{
    uint32_t bytes_written = 0;
    
    // Write delta time
    bytes_written += WriteVariableLength(&buffer[bytes_written], deltaTime);
    
    // Meta event: FF 51 03 (tempo)
    buffer[bytes_written++] = 0xFF;
    buffer[bytes_written++] = 0x51;
    buffer[bytes_written++] = 0x03;
    
    // Tempo in microseconds per quarter note (24-bit big-endian)
    buffer[bytes_written++] = (microsecondsPerQuarter >> 16) & 0xFF;
    buffer[bytes_written++] = (microsecondsPerQuarter >> 8) & 0xFF;
    buffer[bytes_written++] = microsecondsPerQuarter & 0xFF;
    
    return bytes_written;
}

uint32_t MidiFileWriter::WriteEndOfTrack(uint8_t* buffer, uint32_t deltaTime)
{
    uint32_t bytes_written = 0;
    
    // Write delta time
    bytes_written += WriteVariableLength(&buffer[bytes_written], deltaTime);
    
    // Meta event: FF 2F 00 (end of track)
    buffer[bytes_written++] = 0xFF;
    buffer[bytes_written++] = 0x2F;
    buffer[bytes_written++] = 0x00;
    
    return bytes_written;
}

bool MidiFileWriter::GenerateCMajorScaleFile(const char* filename)
{
    FIL file;
    UINT bytes_written;
    FRESULT res;
    
    // Open file for writing
    res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if(res != FR_OK)
    {
        return false;
    }
    
    // Write MIDI header (Format 0, 1 track, 480 ticks per quarter note)
    if(!WriteHeader(&file, 0, 1, 480))
    {
        f_close(&file);
        return false;
    }
    
    // Build track data in memory
    uint8_t track_data[512];  // Should be plenty for a simple scale
    uint32_t track_length = 0;
    
    // Set tempo to 120 BPM (500,000 microseconds per quarter note)
    track_length += WriteTempoEvent(&track_data[track_length], 0, 500000);
    
    // C major scale notes: C, D, E, F, G, A, B, C (MIDI notes 60-72)
    const uint8_t scale_notes[] = {60, 62, 64, 65, 67, 69, 71, 72};
    const uint8_t velocity = 100;
    const uint32_t note_duration = 480;  // Quarter note at 480 ticks per quarter
    
    // Play scale ascending
    for(int i = 0; i < 8; i++)
    {
        // Note on
        track_length += WriteMidiNoteOn(&track_data[track_length], 0, 0, scale_notes[i], velocity);
        
        // Note off after duration
        track_length += WriteMidiNoteOff(&track_data[track_length], note_duration, 0, scale_notes[i], 0);
    }
    
    // Play scale descending (skip the top C as it was just played)
    for(int i = 6; i >= 0; i--)
    {
        // Note on
        track_length += WriteMidiNoteOn(&track_data[track_length], 0, 0, scale_notes[i], velocity);
        
        // Note off after duration
        track_length += WriteMidiNoteOff(&track_data[track_length], note_duration, 0, scale_notes[i], 0);
    }
    
    // End of track
    track_length += WriteEndOfTrack(&track_data[track_length], 0);
    
    // Write MTrk chunk header
    uint8_t track_header[8];
    track_header[0] = 'M';
    track_header[1] = 'T';
    track_header[2] = 'r';
    track_header[3] = 'k';
    WriteBigEndian32(&track_header[4], track_length);
    
    res = f_write(&file, track_header, 8, &bytes_written);
    if(res != FR_OK || bytes_written != 8)
    {
        f_close(&file);
        return false;
    }
    
    // Write track data
    res = f_write(&file, track_data, track_length, &bytes_written);
    if(res != FR_OK || bytes_written != track_length)
    {
        f_close(&file);
        return false;
    }
    
    // Close file
    f_close(&file);
    return true;
}

} // namespace daisy

