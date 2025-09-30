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

TEST(ShiftRegisterMidiTest, SingleNoteOnUsesVoiceZero)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    processor.HandleNoteOn(60, 100);

    ASSERT_EQ(output.messages.size(), 1u);
    const auto& msg = output.messages[0];
    EXPECT_EQ(msg.type, envelope_midi::MidiMessage::Type::kNoteOn);
    EXPECT_EQ(msg.channel, 0u);
    EXPECT_EQ(msg.note, 60u);
    EXPECT_EQ(msg.velocity, 100u);

    const auto& voices = processor.GetVoices();
    EXPECT_TRUE(voices[0].active);
    EXPECT_EQ(voices[0].note, 60u);
    EXPECT_TRUE(voices[0].needs_retrigger);
}

TEST(ShiftRegisterMidiTest, SecondNoteSlidesQueueAndVoices)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    processor.HandleNoteOn(60, 100);
    output.Clear();
    processor.HandleNoteOn(62, 110);

    ASSERT_EQ(output.messages.size(), 3u);
    EXPECT_EQ(output.messages[0].type, envelope_midi::MidiMessage::Type::kNoteOff);
    EXPECT_EQ(output.messages[0].channel, 0u);
    EXPECT_EQ(output.messages[0].note, 60u);

    EXPECT_EQ(output.messages[1].type, envelope_midi::MidiMessage::Type::kNoteOn);
    EXPECT_EQ(output.messages[1].channel, 0u);
    EXPECT_EQ(output.messages[1].note, 62u);

    EXPECT_EQ(output.messages[2].type, envelope_midi::MidiMessage::Type::kNoteOn);
    EXPECT_EQ(output.messages[2].channel, 1u);
    EXPECT_EQ(output.messages[2].note, 60u);

    const auto& voices = processor.GetVoices();
    EXPECT_EQ(voices[0].note, 62u);
    EXPECT_EQ(voices[1].note, 60u);
}

TEST(ShiftRegisterMidiTest, NewestNoteOffReleasesAllGates)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    processor.HandleNoteOn(60, 100);
    processor.HandleNoteOn(62, 110);
    output.Clear();

    // Releasing the newest note (62) releases ALL gates
    processor.HandleNoteOff(62);

    // UpdateAssignments is called, sending MIDI messages
    ASSERT_GT(output.messages.size(), 0u);

    // Both notes should still be in the cascade
    const auto& voices = processor.GetVoices();
    EXPECT_TRUE(voices[0].active);
    EXPECT_EQ(voices[0].note, 62u);
    EXPECT_FALSE(voices[0].gate_on); // ALL gates released!
    
    EXPECT_TRUE(voices[1].active);
    EXPECT_EQ(voices[1].note, 60u);
    EXPECT_FALSE(voices[1].gate_on); // ALL gates released!
}

TEST(ShiftRegisterMidiTest, OlderNoteOffReleasesOnlyThatVoice)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    processor.HandleNoteOn(60, 100);
    processor.HandleNoteOn(62, 110);
    output.Clear();

    // Releasing an older note (60) releases only that voice's gate
    processor.HandleNoteOff(60);

    // UpdateAssignments is called
    ASSERT_GT(output.messages.size(), 0u);

    // Both notes should still be in the cascade
    const auto& voices = processor.GetVoices();
    EXPECT_TRUE(voices[0].active);
    EXPECT_EQ(voices[0].note, 62u);
    EXPECT_TRUE(voices[0].gate_on); // Newest note still held!
    
    EXPECT_TRUE(voices[1].active);
    EXPECT_EQ(voices[1].note, 60u);
    EXPECT_FALSE(voices[1].gate_on); // Only this gate released
}

TEST(ShiftRegisterMidiTest, OverlappingNotesReleaseCorrectly)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    // User scenario: Play A, then play B while holding A
    processor.HandleNoteOn(60, 100); // A
    processor.HandleNoteOn(62, 110); // B (while A held)
    output.Clear();

    // Release B (the newest held note) → should release ALL gates
    processor.HandleNoteOff(62);
    
    const auto& voices_after_b = processor.GetVoices();
    EXPECT_TRUE(voices_after_b[0].active);
    EXPECT_EQ(voices_after_b[0].note, 62u); // B still in cascade
    EXPECT_FALSE(voices_after_b[0].gate_on); // But gate is OFF
    
    EXPECT_TRUE(voices_after_b[1].active);
    EXPECT_EQ(voices_after_b[1].note, 60u); // A still in cascade
    EXPECT_FALSE(voices_after_b[1].gate_on); // Gate also OFF (all released)
    
    output.Clear();

    // Release A → nothing should re-trigger, both should stay OFF
    processor.HandleNoteOff(60);
    
    const auto& voices_after_a = processor.GetVoices();
    EXPECT_TRUE(voices_after_a[0].active); // Still in cascade
    EXPECT_EQ(voices_after_a[0].note, 62u);
    EXPECT_FALSE(voices_after_a[0].gate_on); // Still OFF
    
    EXPECT_TRUE(voices_after_a[1].active);
    EXPECT_EQ(voices_after_a[1].note, 60u);
    EXPECT_FALSE(voices_after_a[1].gate_on); // Still OFF
}

TEST(ShiftRegisterMidiTest, ReleaseNewestHeldAfterOlderReleased)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    // User scenario: Play A, Play B (hold both), Release A, then Release B
    processor.HandleNoteOn(60, 100); // A
    processor.HandleNoteOn(62, 110); // B (while A held)
    output.Clear();

    // Release A (older note) - should only release A's gate
    processor.HandleNoteOff(60);
    
    const auto& voices_after_a = processor.GetVoices();
    EXPECT_TRUE(voices_after_a[0].active);
    EXPECT_EQ(voices_after_a[0].note, 62u); // B
    EXPECT_TRUE(voices_after_a[0].gate_on); // B still held
    
    EXPECT_TRUE(voices_after_a[1].active);
    EXPECT_EQ(voices_after_a[1].note, 60u); // A
    EXPECT_FALSE(voices_after_a[1].gate_on); // A released
    
    output.Clear();

    // Release B (now the only held note, so it's the newest held)
    // This should release B's gate but A should STAY OFF, not re-trigger
    processor.HandleNoteOff(62);
    
    const auto& voices_after_b = processor.GetVoices();
    EXPECT_TRUE(voices_after_b[0].active);
    EXPECT_EQ(voices_after_b[0].note, 62u);
    EXPECT_FALSE(voices_after_b[0].gate_on); // B released
    
    EXPECT_TRUE(voices_after_b[1].active);
    EXPECT_EQ(voices_after_b[1].note, 60u);
    EXPECT_FALSE(voices_after_b[1].gate_on); // A should STAY OFF
}

TEST(ShiftRegisterMidiTest, ResetSendsNoteOffsAndClearsState)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    processor.HandleNoteOn(64, 90);
    output.Clear();

    processor.Reset();

    ASSERT_EQ(output.messages.size(), 1u);
    EXPECT_EQ(output.messages[0].type, envelope_midi::MidiMessage::Type::kNoteOff);
    EXPECT_EQ(output.messages[0].note, 64u);

    for(const auto& voice : processor.GetVoices())
    {
        EXPECT_FALSE(voice.active);
    }
}

TEST(ShiftRegisterMidiTest, DuplicateNoteOnCreatesCanonEffect)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    // First NoteOn 60 - should play on voice 0
    processor.HandleNoteOn(60, 100);
    
    ASSERT_EQ(output.messages.size(), 1u);
    EXPECT_EQ(output.messages[0].type, envelope_midi::MidiMessage::Type::kNoteOn);
    EXPECT_EQ(output.messages[0].channel, 0u);
    EXPECT_EQ(output.messages[0].note, 60u);
    EXPECT_EQ(output.messages[0].velocity, 100u);

    const auto& voices_after_first = processor.GetVoices();
    EXPECT_TRUE(voices_after_first[0].active);
    EXPECT_EQ(voices_after_first[0].note, 60u);
    EXPECT_FALSE(voices_after_first[1].active);

    output.Clear();

    // Second NoteOn 60 - should create canon: voice 0 plays new 60, voice 1 plays old 60
    processor.HandleNoteOn(60, 110);

    // Should send: NoteOff(0,60), NoteOn(0,60), NoteOn(1,60)
    ASSERT_EQ(output.messages.size(), 3u);
    EXPECT_EQ(output.messages[0].type, envelope_midi::MidiMessage::Type::kNoteOff);
    EXPECT_EQ(output.messages[0].channel, 0u);
    EXPECT_EQ(output.messages[0].note, 60u);

    EXPECT_EQ(output.messages[1].type, envelope_midi::MidiMessage::Type::kNoteOn);
    EXPECT_EQ(output.messages[1].channel, 0u);
    EXPECT_EQ(output.messages[1].note, 60u);
    EXPECT_EQ(output.messages[1].velocity, 110u);

    EXPECT_EQ(output.messages[2].type, envelope_midi::MidiMessage::Type::kNoteOn);
    EXPECT_EQ(output.messages[2].channel, 1u);
    EXPECT_EQ(output.messages[2].note, 60u);
    EXPECT_EQ(output.messages[2].velocity, 100u);

    const auto& voices_after_second = processor.GetVoices();
    EXPECT_TRUE(voices_after_second[0].active);
    EXPECT_EQ(voices_after_second[0].note, 60u);
    EXPECT_EQ(voices_after_second[0].velocity, 110u);
    EXPECT_TRUE(voices_after_second[1].active);
    EXPECT_EQ(voices_after_second[1].note, 60u);
    EXPECT_EQ(voices_after_second[1].velocity, 100u);

    output.Clear();

    // NoteOff 60 - the newest instance (voice 0) is released
    // This releases ALL gates since it's the newest overall note
    processor.HandleNoteOff(60);

    // MIDI messages are sent due to UpdateAssignments
    ASSERT_GT(output.messages.size(), 0u);
    
    const auto& voices_after_off = processor.GetVoices();
    // Both instances of note 60 should still be in cascade
    EXPECT_TRUE(voices_after_off[0].active);
    EXPECT_EQ(voices_after_off[0].note, 60u);
    EXPECT_EQ(voices_after_off[0].velocity, 110u);
    EXPECT_FALSE(voices_after_off[0].gate_on); // ALL gates released (newest was released)
    
    EXPECT_TRUE(voices_after_off[1].active);
    EXPECT_EQ(voices_after_off[1].note, 60u);
    EXPECT_EQ(voices_after_off[1].velocity, 100u);
    EXPECT_FALSE(voices_after_off[1].gate_on); // ALL gates released
    
    EXPECT_FALSE(voices_after_off[2].active);
    EXPECT_FALSE(voices_after_off[3].active);
}


