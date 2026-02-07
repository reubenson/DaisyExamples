#include "ScalaTuning.h"
#include <cmath>

// Helper function to convert ratio to cents
float RatioToCents(float ratio) {
    return 1200.0f * log2f(ratio);
}

// Helper function to create tuning from ratios
ScalaTuning CreateTuningFromRatios(const char* name, uint8_t degreeCount, const float* ratios) {
    ScalaTuning tuning;
    tuning.name = name;
    tuning.degreeCount = degreeCount;
    
    for (uint8_t i = 0; i < degreeCount; i++) {
        tuning.degrees[i] = RatioToCents(ratios[i]);
    }
    
    return tuning;
}

// 12-TET (Equal Temperament) - reference tuning
const ScalaTuning TuningPresets::EQUAL_TEMPERAMENT = {
    "12-TET",
    12,
    {0.0f, 100.0f, 200.0f, 300.0f, 400.0f, 500.0f, 600.0f, 700.0f, 800.0f, 900.0f, 1000.0f, 1100.0f}
};

// Pythagorean tuning (3-limit) - based on perfect fifths
const ScalaTuning TuningPresets::PYTHAGOREAN = {
    "Pythagorean",
    12,
    {0.0f, 90.2f, 203.9f, 294.1f, 407.8f, 498.0f, 611.7f, 701.9f, 815.6f, 905.9f, 1019.6f, 1109.8f}
};

// 5-limit just intonation - includes major thirds and perfect fifths
const ScalaTuning TuningPresets::FIVE_LIMIT = {
    "5-Limit",
    12,
    {0.0f, 76.0f, 193.2f, 310.3f, 386.3f, 503.4f, 579.5f, 696.6f, 772.6f, 889.7f, 1006.8f, 1082.9f}
};

// 7-limit just intonation - includes harmonic 7th
const ScalaTuning TuningPresets::SEVEN_LIMIT = {
    "7-Limit",
    12,
    {0.0f, 70.7f, 193.2f, 310.3f, 386.3f, 470.8f, 551.3f, 631.3f, 701.9f, 772.6f, 884.4f, 968.8f}
};

// Quarter-comma meantone - compromise between just intonation and equal temperament
const ScalaTuning TuningPresets::MEANTONE = {
    "Meantone",
    12,
    {0.0f, 76.0f, 193.2f, 310.3f, 386.3f, 503.4f, 579.5f, 696.6f, 772.6f, 889.7f, 1006.8f, 1082.9f}
};

// https://tuning.ableton.com/european-historical/12-wt-sabat-i/
// 17-limit just intonation - includes higher prime factors (17 and 19)
// Based on Scala file with ratios: 1/1, 17/16, 9/8, 81/68, 81/64, 4/3, 27/19, 3/2, 1731/1088, 27/16, 57/32, 243/128, 2/1
const float seventeenLimitRatios[] = {
    1.0f,           // C (unison)
    17.0f/16.0f,    // C♯=17°/C
    9.0f/8.0f,      // D
    81.0f/68.0f,    // E♭=u17\E
    81.0f/64.0f,    // E
    4.0f/3.0f,      // F
    27.0f/19.0f,    // F♯=u19\A
    3.0f/2.0f,      // G
    1731.0f/1088.0f, // G♯|A♭
    27.0f/16.0f,    // A
    57.0f/32.0f,    // B♭=19°/G
    243.0f/128.0f   // B
};

const ScalaTuning TuningPresets::SABAT_I = CreateTuningFromRatios("Sabat-I", 12, seventeenLimitRatios);

// https://tuning.ableton.com/european-historical/12-wt-sabat-ii/
const float sabatIIRatios[] = {
    1.0f,
    16.0f/15.0f, // ^D♭
    9.0f/8.0f, // D
    81.0f/68.0f, // E♭=u17\E
    81.0f/64.0f, // E
    4.0f/3.0f, // F
    729.0f/512.0f, // F♯
    3.0f/2.0f, // G
    51.0f/32.0f, // G♯=17°/G
    27.0f/16.0f, // A
    3645.0f/2048.0f, // vA♯
    243.0f/128.0f, // B
    2.0f/1.0f
};
const ScalaTuning TuningPresets::SABAT_II = CreateTuningFromRatios("Sabat-II", 12, sabatIIRatios);

/*
// Example: How to add more Scala tunings using ratio format
// 
// For a Scala file like:
// 12
// !
// 17/16 ! C♯=17°/C
// 9/8 ! D
// 81/68 ! E♭=u17\E
// ...
//
// Step 1: Extract the ratios (skip the first line which is degree count)
// Step 2: Create ratios array with proper C++ division
// Step 3: Use CreateTuningFromRatios() to create the tuning
// Step 4: Add to TUNING_PRESETS array
//
// Example (see SABAT_I above):
// const float myRatios[] = {1.0f, 17.0f/16.0f, 9.0f/8.0f, ...};
// const ScalaTuning TuningPresets::MY_TUNING = CreateTuningFromRatios("My Tuning", 12, myRatios);
//
// The CreateTuningFromRatios() function automatically converts ratios to cents using RatioToCents()
*/

// Array of all tuning presets
const ScalaTuning TUNING_PRESETS[] = {
    TuningPresets::EQUAL_TEMPERAMENT,
    TuningPresets::PYTHAGOREAN,
    TuningPresets::FIVE_LIMIT,
    TuningPresets::SEVEN_LIMIT,
    TuningPresets::MEANTONE,
    TuningPresets::SABAT_I,
    TuningPresets::SABAT_II
};

const uint8_t NUM_TUNING_PRESETS = sizeof(TUNING_PRESETS) / sizeof(TUNING_PRESETS[0]);

// Helper function to get tuning by index
const ScalaTuning* GetTuningByIndex(uint8_t index) {
    if (index >= NUM_TUNING_PRESETS) {
        return &TUNING_PRESETS[0]; // Return default (12-TET) if index out of range
    }
    return &TUNING_PRESETS[index];
}
