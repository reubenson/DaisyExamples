#pragma once

#include <array>
#include <cstdint>

namespace envelope
{
namespace midi
{

struct MidiMessage
{
    enum class Type
    {
        kNoteOn,
        kNoteOff,
    };

    Type    type     = Type::kNoteOn;
    uint8_t channel  = 0;
    uint8_t note     = 0;
    uint8_t velocity = 0;
};

class MidiOutput
{
  public:
    virtual ~MidiOutput()                           = default;
    virtual void Send(const MidiMessage& message) = 0;
};

struct VoiceState
{
    bool    active          = false;
    uint8_t note            = 0;
    uint8_t velocity        = 0;
    bool    needs_retrigger = false;
    bool    gate_on         = false; // Is the note currently held (for envelope gate)?
};

class ShiftRegisterMidi
{
  public:
    static constexpr size_t kMaxVoices = 4;

    explicit ShiftRegisterMidi(MidiOutput* output = nullptr);

    void SetOutput(MidiOutput* output);
    void Reset();

    void HandleNoteOn(uint8_t note, uint8_t velocity);
    void HandleNoteOff(uint8_t note);

    const std::array<VoiceState, kMaxVoices>& GetVoices() const;
    void                                      ClearRetriggerFlags();

  private:
    struct QueueEntry
    {
        uint8_t note     = 0;
        uint8_t velocity = 0;
        bool    active   = false;
        bool    held     = false; // Is the note currently held (gate on)?
    };

    void UpdateAssignments();
    void SendNoteOn(size_t voice_index, uint8_t note, uint8_t velocity) const;
    void SendNoteOff(size_t voice_index, uint8_t note) const;

    MidiOutput*                                 output_;
    std::array<QueueEntry, kMaxVoices>          queue_;
    std::array<VoiceState, kMaxVoices>          voices_;
};

} // namespace midi
} // namespace envelope


