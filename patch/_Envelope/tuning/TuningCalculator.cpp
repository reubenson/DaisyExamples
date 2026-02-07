#include "TuningCalculator.h"
#include <cmath>

float CalculateCentsDeviation(uint8_t midiNote, const ScalaTuning* tuning) {
    if (!tuning || tuning->degreeCount == 0) {
        return 0.0f; // Return no deviation if tuning is invalid
    }
    
    // Use MIDI note 60 (middle C) as reference
    const uint8_t referenceNote = 60;
    
    // Calculate semitone difference from reference
    int semitoneDiff = midiNote - referenceNote;
    
    // Handle octave wrapping for scales with fewer than 12 degrees
    int octave = semitoneDiff / 12;
    int degreeInOctave = semitoneDiff % 12;
    
    // Ensure positive degree index
    while (degreeInOctave < 0) {
        degreeInOctave += 12;
        octave--;
    }
    
    // Calculate the cents value for this degree
    float centsValue = 0.0f;
    
    if (tuning->degreeCount == 12) {
        // Standard 12-degree scale
        centsValue = tuning->degrees[degreeInOctave];
    } else {
        // Non-standard scale - interpolate or map to closest degree
        // For simplicity, map to closest degree in the scale
        float degreeRatio = (float)degreeInOctave / 12.0f;
        int targetDegree = (int)(degreeRatio * tuning->degreeCount);
        
        if (targetDegree >= tuning->degreeCount) {
            targetDegree = tuning->degreeCount - 1;
        }
        
        centsValue = tuning->degrees[targetDegree];
    }
    
    // Add octave offset (1200 cents per octave)
    centsValue += octave * 1200.0f;
    
    // Calculate deviation from equal temperament
    float equalTemperamentCents = semitoneDiff * 100.0f;
    float deviation = centsValue - equalTemperamentCents;
    
    return deviation;
}

int16_t CentsToPitchBend(float centsDeviation, float pitchBendRange) {
    // Clamp deviation to pitch bend range
    if (centsDeviation > pitchBendRange) {
        centsDeviation = pitchBendRange;
    } else if (centsDeviation < -pitchBendRange) {
        centsDeviation = -pitchBendRange;
    }
    
    // Convert to 14-bit pitch bend value
    // Range: -pitchBendRange to +pitchBendRange cents
    // MIDI range: 0 to 16383 (8192 = center)
    float normalizedValue = centsDeviation / pitchBendRange; // -1.0 to 1.0
    int16_t pitchBendValue = (int16_t)(8192 + normalizedValue * 8191);
    
    // Clamp to valid MIDI range
    if (pitchBendValue < 0) {
        pitchBendValue = 0;
    } else if (pitchBendValue > 16383) {
        pitchBendValue = 16383;
    }
    
    return pitchBendValue;
}

float CalculateFrequencyMultiplier(uint8_t midiNote, const ScalaTuning* tuning) {
    if (!tuning || tuning->degreeCount == 0) {
        return 1.0f; // No adjustment if tuning is invalid
    }
    
    // Calculate cents deviation
    float centsDeviation = CalculateCentsDeviation(midiNote, tuning);
    
    // Convert cents to frequency multiplier
    // cents = 1200 * log2(freq_ratio)
    // freq_ratio = 2^(cents/1200)
    float frequencyMultiplier = powf(2.0f, centsDeviation / 1200.0f);
    
    return frequencyMultiplier;
}
