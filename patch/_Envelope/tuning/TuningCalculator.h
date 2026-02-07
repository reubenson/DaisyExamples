#pragma once

#include "ScalaTuning.h"
#include <stdint.h>

// Calculate cents deviation from equal temperament for a given MIDI note
// Reference: MIDI note 60 (middle C) = 0 cents deviation
// Returns deviation in cents from 12-TET
float CalculateCentsDeviation(uint8_t midiNote, const ScalaTuning* tuning);

// Convert cents deviation to MIDI pitch bend value
// pitchBendRange: range in cents (e.g., 200 for ±200 cents)
// Returns 14-bit pitch bend value (0-16383, 8192 = center)
int16_t CentsToPitchBend(float centsDeviation, float pitchBendRange);

// Calculate frequency adjustment factor for internal oscillators
// Returns multiplier to apply to base frequency
float CalculateFrequencyMultiplier(uint8_t midiNote, const ScalaTuning* tuning);
