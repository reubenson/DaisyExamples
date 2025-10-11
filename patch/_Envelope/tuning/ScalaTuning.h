#pragma once

#include <stdint.h>

// Maximum number of degrees in a Scala tuning
#define MAX_SCALA_DEGREES 128

// Structure to hold a Scala tuning
struct ScalaTuning {
    const char* name;                    // Human-readable name
    uint8_t degreeCount;                 // Number of degrees in the scale
    float degrees[MAX_SCALA_DEGREES];     // Cents values for each degree
};

// Embedded tuning presets - common just intonation scales
extern const ScalaTuning TUNING_PRESETS[];
extern const uint8_t NUM_TUNING_PRESETS;

// Helper function to get tuning by index
const ScalaTuning* GetTuningByIndex(uint8_t index);

// Utility functions for converting Scala files
float RatioToCents(float ratio);
ScalaTuning CreateTuningFromRatios(const char* name, uint8_t degreeCount, const float* ratios);

// Common just intonation scales embedded as data
namespace TuningPresets {
    // 12-TET (Equal Temperament) - reference
    extern const ScalaTuning EQUAL_TEMPERAMENT;
    
    // Pythagorean tuning (3-limit)
    extern const ScalaTuning PYTHAGOREAN;
    
    // 5-limit just intonation
    extern const ScalaTuning FIVE_LIMIT;
    
    // 7-limit just intonation
    extern const ScalaTuning SEVEN_LIMIT;
    
    // Quarter-comma meantone
    extern const ScalaTuning MEANTONE;
    
    // 17-limit just intonation
    extern const ScalaTuning SEVENTEEN_LIMIT;
    
    // Sabat tunings from European historical temperaments
    extern const ScalaTuning SABAT_I;
    extern const ScalaTuning SABAT_II;
}
