#include <gtest/gtest.h>
#include <vector>
#include "ShiftRegisterMidi.h"

namespace envelope_midi = envelope::midi;

class CollectingOutput : public envelope_midi::MidiOutput
{
  public:
    void Send(const envelope_midi::MidiMessage& message) override
    {
        messages.push_back(message);
    }

    void Clear() { messages.clear(); }

    std::vector<envelope_midi::MidiMessage> messages;
};

// This test simulates what happens when you play notes one at a time
// (press, release, press, release) like normal keyboard playing
// In persistent canon mode, notes stay in cascade but gates respond naturally
TEST(SequentialNotesTest, SequentialNotesBuildPersistentCanon)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    // Play note 60 and release it
    processor.HandleNoteOn(60, 100);
    output.Clear();
    processor.HandleNoteOff(60); // Releases gate but keeps in cascade
    
    output.Clear();
    
    // Play note 62
    processor.HandleNoteOn(62, 110);
    
    // In persistent mode, both notes should be in the cascade
    const auto& voices = processor.GetVoices();
    
    // Canon is built from sequential notes!
    EXPECT_TRUE(voices[0].active);
    EXPECT_EQ(voices[0].note, 62u);
    EXPECT_TRUE(voices[0].gate_on); // New note has gate on
    
    EXPECT_TRUE(voices[1].active);
    EXPECT_EQ(voices[1].note, 60u);
    EXPECT_TRUE(voices[1].gate_on); // Re-triggered when 62 was pressed!
}
