#include "ShiftRegisterMidi.h"

namespace envelope
{
namespace midi
{

ShiftRegisterMidi::ShiftRegisterMidi(MidiOutput* output)
: output_(output)
{
    Reset();
}

void ShiftRegisterMidi::SetOutput(MidiOutput* output)
{
    output_ = output;
}

void ShiftRegisterMidi::Reset()
{
    for(size_t i = 0; i < voices_.size(); ++i)
    {
        if(voices_[i].active)
        {
            SendNoteOff(i, voices_[i].note);
        }
        voices_[i] = VoiceState{};
    }

    for(auto& entry : queue_)
    {
        entry = QueueEntry{};
    }
}

void ShiftRegisterMidi::HandleNoteOn(uint8_t note, uint8_t velocity)
{
    for(size_t i = 0; i + 1 < queue_.size(); ++i)
    {
        queue_[i] = queue_[i + 1];
    }

    queue_[queue_.size() - 1] = QueueEntry{note, velocity, true};

    UpdateAssignments();
}

void ShiftRegisterMidi::HandleNoteOff(uint8_t note)
{
    bool removed = false;
    for(size_t i = 0; i < queue_.size(); ++i)
    {
        if(!removed && queue_[i].active && queue_[i].note == note)
        {
            removed = true;
        }

        if(removed && i + 1 < queue_.size())
        {
            queue_[i] = queue_[i + 1];
        }
    }

    if(removed)
    {
        queue_[queue_.size() - 1] = QueueEntry{};
        // Compact queue so remaining notes occupy the most recent slots
        size_t active_count = 0;
        for(const auto& entry : queue_)
        {
            if(entry.active)
            {
                ++active_count;
            }
        }

        std::array<QueueEntry, kMaxVoices> compacted{};
        if(active_count > 0)
        {
            size_t start_index = kMaxVoices - active_count;
            size_t current     = start_index;
            for(const auto& entry : queue_)
            {
                if(entry.active)
                {
                    compacted[current++] = entry;
                }
            }
        }

        queue_ = compacted;
    }

    UpdateAssignments();
}

const std::array<VoiceState, ShiftRegisterMidi::kMaxVoices>& ShiftRegisterMidi::GetVoices() const
{
    return voices_;
}

void ShiftRegisterMidi::ClearRetriggerFlags()
{
    for(auto& voice : voices_)
    {
        voice.needs_retrigger = false;
    }
}

void ShiftRegisterMidi::UpdateAssignments()
{
    for(size_t voice_index = 0; voice_index < voices_.size(); ++voice_index)
    {
        if(voices_[voice_index].active)
        {
            SendNoteOff(voice_index, voices_[voice_index].note);
        }
        voices_[voice_index] = VoiceState{};
    }

    for(size_t voice_index = 0; voice_index < voices_.size(); ++voice_index)
    {
        size_t queue_index = queue_.size() - 1 - voice_index;
        const QueueEntry& entry = queue_[queue_index];
        if(!entry.active)
        {
            continue;
        }

        voices_[voice_index].active         = true;
        voices_[voice_index].note           = entry.note;
        voices_[voice_index].velocity       = entry.velocity;
        voices_[voice_index].needs_retrigger = true;

        SendNoteOn(voice_index, entry.note, entry.velocity);
    }
}

void ShiftRegisterMidi::SendNoteOn(size_t voice_index, uint8_t note, uint8_t velocity) const
{
    if(!output_)
    {
        return;
    }

    MidiMessage message;
    message.type     = MidiMessage::Type::kNoteOn;
    message.channel  = static_cast<uint8_t>(voice_index);
    message.note     = note;
    message.velocity = velocity;
    output_->Send(message);
}

void ShiftRegisterMidi::SendNoteOff(size_t voice_index, uint8_t note) const
{
    if(!output_)
    {
        return;
    }

    MidiMessage message;
    message.type     = MidiMessage::Type::kNoteOff;
    message.channel  = static_cast<uint8_t>(voice_index);
    message.note     = note;
    message.velocity = 0;
    output_->Send(message);
}

} // namespace midi
} // namespace envelope


