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

    QueueEntry new_entry;
    new_entry.note     = note;
    new_entry.velocity = velocity;
    new_entry.active   = true;
    new_entry.held     = true; // Mark as currently held
    
    queue_[queue_.size() - 1] = new_entry;

    // Re-trigger ALL active notes in the cascade
    for(auto& entry : queue_)
    {
        if(entry.active)
        {
            entry.held = true;
        }
    }

    UpdateAssignments();
}

void ShiftRegisterMidi::HandleNoteOff(uint8_t note)
{
    // Hierarchical gate control:
    // - If the newest HELD note is released → release ALL gates
    // - If an older note is released → release only that specific note's gate
    
    // Find the newest matching note (search backwards)
    int newest_match_index = -1;
    for(int i = static_cast<int>(queue_.size()) - 1; i >= 0; --i)
    {
        if(queue_[i].active && queue_[i].note == note && queue_[i].held)
        {
            newest_match_index = i;
            break;
        }
    }
    
    if(newest_match_index == -1)
    {
        return; // Note not found or already released
    }
    
    // Find the newest HELD note in the entire queue (not just matching notes)
    int newest_held_index = -1;
    for(int i = static_cast<int>(queue_.size()) - 1; i >= 0; --i)
    {
        if(queue_[i].active && queue_[i].held)
        {
            newest_held_index = i;
            break;
        }
    }
    
    // Check if the note being released is the newest HELD note
    bool is_newest_held = (newest_match_index == newest_held_index);
    
    if(is_newest_held)
    {
        // Release ALL gates when the newest held note is released
        for(auto& entry : queue_)
        {
            if(entry.active)
            {
                entry.held = false;
            }
        }
    }
    else
    {
        // Release only this specific note's gate
        queue_[newest_match_index].held = false;
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

        voices_[voice_index].active          = true;
        voices_[voice_index].note            = entry.note;
        voices_[voice_index].velocity        = entry.velocity;
        voices_[voice_index].needs_retrigger = entry.held; // Only retrigger if gate is on
        voices_[voice_index].gate_on         = entry.held; // Copy gate state

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


