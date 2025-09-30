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

TEST(ShiftRegisterMidiTest, NoteOffReassignsRemainingVoices)
{
    CollectingOutput output;
    envelope_midi::ShiftRegisterMidi processor(&output);

    processor.HandleNoteOn(60, 100);
    processor.HandleNoteOn(62, 110);
    output.Clear();

    processor.HandleNoteOff(62);

    ASSERT_EQ(output.messages.size(), 3u);
    EXPECT_EQ(output.messages[0].type, envelope_midi::MidiMessage::Type::kNoteOff);
    EXPECT_EQ(output.messages[0].channel, 0u);
    EXPECT_EQ(output.messages[0].note, 62u);

    EXPECT_EQ(output.messages[1].type, envelope_midi::MidiMessage::Type::kNoteOff);
    EXPECT_EQ(output.messages[1].channel, 1u);
    EXPECT_EQ(output.messages[1].note, 60u);

    EXPECT_EQ(output.messages[2].type, envelope_midi::MidiMessage::Type::kNoteOn);
    EXPECT_EQ(output.messages[2].channel, 0u);
    EXPECT_EQ(output.messages[2].note, 60u);

    const auto& voices = processor.GetVoices();
    EXPECT_TRUE(voices[0].active);
    EXPECT_FALSE(voices[1].active);
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


