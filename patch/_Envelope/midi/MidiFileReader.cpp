#include "MidiFileReader.h"
#include "hid/MidiEvent.h"
#include <algorithm>
#include <cstring>

namespace daisy
{

bool MidiFileReader::LoadFile(const char* filename)
{
    FIL file;
    FRESULT res;
    
    // Clear previous data
    events_.clear();
    currentEventIndex_ = 0;
    totalTicks_ = 0;
    fileLoaded_ = false;
    
    // Open file for reading
    res = f_open(&file, filename, FA_READ);
    if(res != FR_OK)
    {
        return false;
    }
    
    // Read file into memory buffer
    const uint32_t MAX_FILE_SIZE = 32768; // 32KB max file size
    uint8_t* fileBuffer = new uint8_t[MAX_FILE_SIZE];
    UINT bytesRead;
    
    res = f_read(&file, fileBuffer, MAX_FILE_SIZE, &bytesRead);
    f_close(&file);
    
    if(res != FR_OK || bytesRead < 14) // Minimum header size
    {
        delete[] fileBuffer;
        return false;
    }
    
    // Parse MIDI file
    uint32_t offset = 0;
    
    // Parse header chunk
    if(bytesRead < 14 || memcmp(fileBuffer, "MThd", 4) != 0)
    {
        delete[] fileBuffer;
        return false;
    }
    
    if(!ParseHeader(fileBuffer))
    {
        delete[] fileBuffer;
        return false;
    }
    
    offset = 14; // Skip header
    
    // Parse track chunks
    while(offset < bytesRead - 8)
    {
        if(memcmp(&fileBuffer[offset], "MTrk", 4) == 0)
        {
            uint32_t trackLength = ReadBigEndian32(&fileBuffer[offset + 4]);
            offset += 8;
            
            if(offset + trackLength <= bytesRead)
            {
                ParseTrack(&fileBuffer[offset], trackLength);
                offset += trackLength;
            }
            else
            {
                break; // Invalid track length
            }
        }
        else
        {
            break; // Unknown chunk type
        }
    }
    
    // Sort events by tick time
    std::sort(events_.begin(), events_.end(), 
        [](const MidiEventData& a, const MidiEventData& b) {
            return a.tick < b.tick;
        });
    
    // Find total ticks
    if(!events_.empty())
    {
        totalTicks_ = events_.back().tick;
    }
    
    delete[] fileBuffer;
    fileLoaded_ = true;
    return true;
}

bool MidiFileReader::GetNextEvent(uint32_t currentTick, MidiEvent* event)
{
    if(!fileLoaded_ || events_.empty())
    {
        return false;
    }
    
    // Find events at current tick
    while(currentEventIndex_ < events_.size())
    {
        const MidiEventData& eventData = events_[currentEventIndex_];
        
        if(eventData.tick == currentTick)
        {
            // Fill output event
            event->channel = eventData.channel;
            event->data[0] = eventData.data1;
            event->data[1] = eventData.data2;
            
            if(eventData.isNoteOn)
            {
                event->type = NoteOn;
            }
            else if(eventData.status == 0x80 || eventData.status == 0x90)
            {
                event->type = NoteOff;
            }
            else if(eventData.status >= 0xB0 && eventData.status < 0xC0)
            {
                event->type = ControlChange;
            }
            else
            {
                event->type = NoteOff; // Default fallback
            }
            
            currentEventIndex_++;
            return true;
        }
        else if(eventData.tick > currentTick)
        {
            break; // No more events at this tick
        }
        else
        {
            currentEventIndex_++; // Skip past events
        }
    }
    
    return false;
}

void MidiFileReader::Reset()
{
    currentEventIndex_ = 0;
}

uint32_t MidiFileReader::ReadBigEndian32(const uint8_t* buffer)
{
    return (static_cast<uint32_t>(buffer[0]) << 24) |
           (static_cast<uint32_t>(buffer[1]) << 16) |
           (static_cast<uint32_t>(buffer[2]) << 8) |
           static_cast<uint32_t>(buffer[3]);
}

uint16_t MidiFileReader::ReadBigEndian16(const uint8_t* buffer)
{
    return (static_cast<uint16_t>(buffer[0]) << 8) |
           static_cast<uint16_t>(buffer[1]);
}

uint32_t MidiFileReader::ReadVariableLength(const uint8_t* buffer, uint32_t* bytesRead)
{
    uint32_t value = 0;
    uint32_t count = 0;
    
    do
    {
        value = (value << 7) | (buffer[count] & 0x7F);
        count++;
    } while((buffer[count - 1] & 0x80) != 0 && count < 4);
    
    *bytesRead = count;
    return value;
}

bool MidiFileReader::ParseHeader(const uint8_t* buffer)
{
    // Skip "MThd" and chunk length
    uint16_t format = ReadBigEndian16(&buffer[8]);
    // uint16_t tracks = ReadBigEndian16(&buffer[10]);  // Not used currently
    ticksPerQuarterNote_ = ReadBigEndian16(&buffer[12]);
    
    // Only support Format 0 and 1
    if(format > 1)
    {
        return false;
    }
    
    return true;
}

bool MidiFileReader::ParseTrack(const uint8_t* buffer, uint32_t length)
{
    uint32_t offset = 0;
    uint32_t currentTick = 0;
    uint8_t runningStatus = 0;
    
    while(offset < length)
    {
        // Read delta time
        uint32_t deltaBytes;
        uint32_t deltaTime = ReadVariableLength(&buffer[offset], &deltaBytes);
        offset += deltaBytes;
        currentTick += deltaTime;
        
        if(offset >= length)
        {
            break;
        }
        
        uint8_t status = buffer[offset];
        
        // Handle meta events and system exclusive
        if(status == 0xFF)
        {
            // Meta event - skip it
            offset++;
            if(offset >= length) break;
            
            // uint8_t metaType = buffer[offset++];  // Not used currently
            offset++;
            if(offset >= length) break;
            
            uint32_t metaLengthBytes;
            uint32_t metaLength = ReadVariableLength(&buffer[offset], &metaLengthBytes);
            offset += metaLengthBytes + metaLength;
            
            if(offset > length) break;
            continue;
        }
        else if(status == 0xF0 || status == 0xF7)
        {
            // System exclusive - skip it
            offset++;
            if(offset >= length) break;
            
            uint32_t sysexLengthBytes;
            uint32_t sysexLength = ReadVariableLength(&buffer[offset], &sysexLengthBytes);
            offset += sysexLengthBytes + sysexLength;
            
            if(offset > length) break;
            continue;
        }
        
        // Handle MIDI events
        uint8_t eventStatus = status;
        uint8_t channel = status & 0x0F;
        
        // Check for running status
        if((status & 0x80) == 0)
        {
            // Running status - use previous status
            eventStatus = runningStatus;
            channel = runningStatus & 0x0F;
            offset--; // Back up to read the first data byte
        }
        else
        {
            runningStatus = status;
        }
        
        // Parse event based on status
        if((eventStatus & 0xF0) == 0x80 || (eventStatus & 0xF0) == 0x90)
        {
            // Note On or Note Off
            if(offset + 2 > length) break;
            
            uint8_t note = buffer[offset++];
            uint8_t velocity = buffer[offset++];
            
            MidiEventData eventData;
            eventData.tick = currentTick;
            eventData.status = eventStatus;
            eventData.data1 = note;
            eventData.data2 = velocity;
            eventData.channel = channel;
            eventData.isNoteOn = ((eventStatus & 0xF0) == 0x90) && (velocity > 0);
            
            events_.push_back(eventData);
        }
        else if((eventStatus & 0xF0) == 0xB0)
        {
            // Control Change
            if(offset + 2 > length) break;
            
            uint8_t controller = buffer[offset++];
            uint8_t value = buffer[offset++];
            
            MidiEventData eventData;
            eventData.tick = currentTick;
            eventData.status = eventStatus;
            eventData.data1 = controller;
            eventData.data2 = value;
            eventData.channel = channel;
            eventData.isNoteOn = false;
            
            events_.push_back(eventData);
        }
        else if((eventStatus & 0xF0) == 0xC0)
        {
            // Program Change
            if(offset + 1 > length) break;
            
            uint8_t program = buffer[offset++];
            
            MidiEventData eventData;
            eventData.tick = currentTick;
            eventData.status = eventStatus;
            eventData.data1 = program;
            eventData.data2 = 0;
            eventData.channel = channel;
            eventData.isNoteOn = false;
            
            events_.push_back(eventData);
        }
        else if((eventStatus & 0xF0) == 0xE0)
        {
            // Pitch Bend
            if(offset + 2 > length) break;
            
            uint8_t lsb = buffer[offset++];
            uint8_t msb = buffer[offset++];
            
            MidiEventData eventData;
            eventData.tick = currentTick;
            eventData.status = eventStatus;
            eventData.data1 = lsb;
            eventData.data2 = msb;
            eventData.channel = channel;
            eventData.isNoteOn = false;
            
            events_.push_back(eventData);
        }
        else
        {
            // Unknown event - skip it
            offset++;
        }
    }
    
    return true;
}

} // namespace daisy
