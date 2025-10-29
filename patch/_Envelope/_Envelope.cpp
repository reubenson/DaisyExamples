#include "daisy_patch.h"
#include "daisysp.h"
#include <algorithm>
#include <string>
#include <cmath>
#include <vector>
#include <cstdarg>
#include "midi/ShiftRegisterMidi.h"
#include "hid/parameter.h"
#include "tuning/ScalaTuning.h"
#include "tuning/TuningCalculator.h"
#include "ScreenUtils.h"
#include "util/bsp_sd_diskio.h"

// Font aliases
#define font_s Font_6x8    // 6x8 pixels - small, good for labels and compact info
#define font_m Font_7x10   // 7x10 pixels - medium, good for panel names
#define font_l Font_11x18  // 11x18 pixels - large, good for emphasis

using namespace daisy;
using namespace daisysp;

DaisyPatch      hw;
Fm2             osc1, osc2;
Oscillator      pan, lfo1, lfo2, lfo3;
SdmmcHandler    sdcard;
Compressor      compressor;

// Custom oscillator class for waveform interpolation
class InterpolatedOscillator {
private:
    float phase_;
    float freq_;
    float amp_;
    float waveform_param_; // 0.0 = sine, 0.33 = triangle, 0.66 = square, 1.0 = saw
    float sample_rate_;
    
public:
    InterpolatedOscillator() : phase_(0.0f), freq_(440.0f), amp_(1.0f), waveform_param_(0.0f), sample_rate_(48000.0f) {}
    
    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        phase_ = 0.0f;
    }
    
    void SetFreq(float freq) {
        freq_ = freq;
    }
    
    void SetAmp(float amp) {
        amp_ = amp;
    }
    
    void SetWaveformParam(float param) {
        waveform_param_ = param;
    }
    
    float Process() {
        float output = 0.0f;
        
        // Generate base waveforms
        float sine = sinf(phase_ * 2.0f * M_PI);
        float triangle = 2.0f * (phase_ < 0.5f ? 2.0f * phase_ : 2.0f * (1.0f - phase_)) - 1.0f;
        float square = phase_ < 0.5f ? 1.0f : -1.0f;
        float saw = 2.0f * phase_ - 1.0f;
        
        // Interpolate between waveforms
        if (waveform_param_ <= 0.33f) {
            // Interpolate between sine and triangle
            float t = waveform_param_ / 0.33f;
            output = sine * (1.0f - t) + triangle * t;
        } else if (waveform_param_ <= 0.66f) {
            // Interpolate between triangle and square
            float t = (waveform_param_ - 0.33f) / 0.33f;
            output = triangle * (1.0f - t) + square * t;
        } else {
            // Interpolate between square and saw
            float t = (waveform_param_ - 0.66f) / 0.34f;
            output = square * (1.0f - t) + saw * t;
        }
        
        // Update phase
        phase_ += freq_ / sample_rate_;
        if (phase_ >= 1.0f) {
            phase_ -= 1.0f;
        }
        
        return output * amp_;
    }
};

InterpolatedOscillator voiceInterpOsc[4];  // Oscillators for all 4 voices
Fm2 voiceFm2Osc[4];                        // FM2 oscillators for all 4 voices
FormantOscillator voiceFormantOsc[4];             // Formant oscillators for all 4 voices
HarmonicOscillator<> voiceHarmonicOsc[4];    // Harmonic oscillators for all 4 voices (default 16 harmonics)

int panelMode;
float voicesMinLevel = 0.0f;
float cvOut1;
float cvOut2;
float panOutput;
float panPhase = 0.0f;  // Manual phase tracking for quadrature panning
float panFreq = 0.2f;   // Pan LFO frequency (0-10Hz range)
float panAmp = 1.0f;    // Pan amplitude (0 = centered, 1 = full panning)
int8_t currentHighestNote = 0;
int8_t lastHighestNote = 0;
int8_t currentLowestNote = 0;
int8_t lastLowestNote = 0;
int8_t currentNote = 0;
int8_t lastCurrentNote = 0;
int8_t nextVoiceIndex = 0;  // Round-robin voice allocator (0-3)
uint32_t voiceAllocationCounter = 0;  // Counter to track voice allocation order

bool shiftRegisterMode = false;

// option to use internal oscillators for each voice
bool useInternalOscillators[4] = {true, true, true, true};

// Signal detection state
bool signalDetectionEnabled = false;
uint32_t signalDetectionSampleCount = 0;
const uint32_t SIGNAL_DETECTION_SAMPLES = 4800;  // ~100ms at 48kHz
float signalDetectionAccum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float signalDetectionPeak[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Tuning system variables
uint8_t currentTuningIndex = 0;  // Current tuning preset index
// this normalizedValue currently corresponds to the 2 semitone pitch bend range configured on Intellijel
float pitchBendRange = 200.0f;    // Pitch bend range in cents (default ±200)
bool sendPitchBendMidi = true;    // Enable/disable pitch bend MIDI output
bool applyToInternalOsc = true;   // Enable/disable tuning for internal oscillators
int16_t currentPitchBendValues[16]; // Track pitch bend per channel (8192 = center)

namespace envelope_midi = envelope::midi;

struct DaisyMidiOutput : public envelope_midi::MidiOutput
{
    void Send(const envelope_midi::MidiMessage& message) override
    {
        uint8_t status_base =
            (message.type == envelope_midi::MidiMessage::Type::kNoteOn) ? 0x90 : 0x80;
        uint8_t bytes[3] = {
            static_cast<uint8_t>(status_base + message.channel),
            message.note,
            message.velocity,
        };
        hw.midi.SendMessage(bytes, 3);
    }
};

static DaisyMidiOutput               daisy_midi_output;
static envelope_midi::ShiftRegisterMidi shift_register(&daisy_midi_output);

static void ApplyShiftRegisterState();

// ============================================================================
// MIDI Architecture - Template-based Handler Chain (Zero Runtime Overhead)
// ============================================================================
// Forward declarations
struct NullHandler;
template<typename NextHandler> class HandlerBase;
template<typename NextHandler> class SequencerCaptureHandler;
template<typename NextHandler> class ShiftRegisterHandler;
template<typename NextHandler> class IntellijelTrackerHandler;
template<typename NextHandler> class NormalVoiceHandler;

void ProcessHandlerChainNoteOn(NoteOnEvent& event);
void ProcessHandlerChainNoteOff(NoteOffEvent& event);
void ProcessSequencerMidiSource();
void ResetCCState(); // Reset CC state to clean state

// Display update timing
uint32_t lastDisplayUpdate = 0;
const uint32_t DISPLAY_UPDATE_INTERVAL_MS = 100; // Update display every 100ms

// Debug message system
const size_t DEBUG_MESSAGE_SIZE = 32;  // Standard debug message buffer size
char debugMessage[DEBUG_MESSAGE_SIZE] = "";  // Buffer for debug message
bool debugMessageActive = false;  // Whether debug message should be displayed
uint32_t debugMessageTime = 0;  // When debug message was set
const uint32_t DEBUG_MESSAGE_DURATION_MS = 2000;  // How long to show debug message (2 seconds)

// Trigger off timing
// uint32_t triggerOffTime = 0;
// const uint32_t TRIGGER_OFF_DELAY_MS = 50;
// bool triggerOffPending = false;

// CC-triggered trigger timing (separate from note-triggered triggers)
uint32_t ccTriggerOffTime = 0;
bool ccTriggerOffPending = false;

// CC reset timing - for channel assignment on channel 16
// to experimentally test for normalizedValue - send trig to next input and confirm 8 pulses
const uint32_t CC_RESET_DELAY_MS = 15; // drops signal sometimes below 15ms
uint8_t lastCCValue = 0; // Track the last CC normalizedValue sent (start with lowest CC normalizedValue)
bool ccStateInitialized = false; // Track if CC state has been properly initialized

// CC Subdivision System variables
uint8_t ccSlotValues[8] = {0, 18, 36, 54, 73, 91, 109, 127}; // Equally distributed CC normalizedValues 0-127
uint8_t ccSlotProbabilities[8] = {100, 100, 100, 100, 100, 100, 100, 100}; // Default all probabilities to 100%
uint8_t ccSlotMultipliers[8] = {1, 1, 1, 1, 1, 1, 1, 1}; // Multiplier for each slot (1, 2, 4, or 8)
uint8_t ccSlotCounters[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Track note count for each slot
uint32_t globalNoteCounter = 0; // Increments on every note-on

// CC Queue System
struct CCQueueItem {
    uint8_t ccValue;
    uint32_t sendTime;
    uint32_t holdUntil; // Time when this CC should be released
    bool isReset; // true if this is a reset to lowest normalizedValue
};

const size_t CC_QUEUE_SIZE = 16; // Reduced from 16 to save memory
CCQueueItem ccQueue[CC_QUEUE_SIZE];
size_t ccQueueHead = 0;
size_t ccQueueTail = 0;
size_t ccQueueCount = 0;
uint32_t ccLatchTime = 0; // Time when CC was last sent
bool ccIsLatched = false; // Whether CC is currently latched to a normalizedValue

// Encoder long press timing
const float ENCODER_SEQUENCER_TOGGLE_MS = 500.0f;
bool encoderWasPressed = false;
bool longPressHandled = false;

// MIDI Clock variables
const int32_t CLOCK_BPM_MIN = 10;     // Minimum BPM
const int32_t CLOCK_BPM_MAX = 600;   // Maximum BPM
const int32_t CLOCK_BPM_DEFAULT = 120; // Default BPM
const int32_t CLOCK_BPM_INCREMENT = 5; // BPM change per encoder tick

int8_t encoderIncrement = 0;  // Track encoder rotation

// Trigger Sequence Generator variables
const uint8_t TRIGGER_SEQUENCE_LENGTH = 16;  // 16-step sequence

// Trigger off timing for sequence
// uint32_t sequenceTriggerOffTime = 0;
// bool sequenceTriggerOffPending = false;


// Trigger Sequence Generator functions
void      InitTriggerSequence();
void      GenerateEuclideanRhythm(int numTriggers, int numSteps, bool* pattern);
void      BuildPattern(int level, std::vector<bool>& result, const std::vector<int>& count, const std::vector<int>& remainder);

// Sequencer parameters and state
struct SequencerParams {
    // Hardware parameter objects
    // Parameter noteParam;
    
    // Density property (0.0f to 16.0f)
    float density = 0.0f;
    
    // Sequencer mode and state
    bool sequencerMode = false;
    bool sequencerNotesAscending = true;
    uint8_t sequencerNoteIndex = 0;
    float sequencerNoteLengthPercent = 0.5f;  // Note length as percentage of step (10%-90%)
    bool sequencerUsingInitialCapture = false;
    
    // Sequencer note management
    std::vector<uint8_t> sequencerNotes;  // Array of held notes for sequencer
    uint32_t sequencerNoteOffTime = 0;
    bool sequencerNoteOffPending = false;
    uint8_t sequencerNoteToTurnOff = 0;
    int8_t sequencerVoiceToTurnOff = -1;
    
    // Clock and timing
    int32_t clockBpm = CLOCK_BPM_DEFAULT;
    uint32_t lastClockTime = 0;
    uint32_t clockInterval = 0;
    bool clockEnabled = true;
    
    // Trigger sequence
    bool triggerSequence[TRIGGER_SEQUENCE_LENGTH] = {false};
    uint8_t currentSequenceStep = 0;
    uint32_t lastSequenceStepTime = 0;
    uint32_t sequenceStepInterval = 0;
    // bool sequenceEnabled = true;
    uint8_t triggerNote = 36;  // MIDI note for triggers (C2)
    uint8_t ccTriggerChannel = 12;
    uint8_t ccValueChannel = 15;
    
    // Subdivision timing for CC multiplier feature
    uint8_t currentSubdivision = 0; // 0-7, tracks which of 8 subdivisions we're on
    uint32_t lastSubdivisionTime = 0;
    uint32_t subdivisionInterval = 0; // Calculated as sequenceStepInterval / 8
    
    // Inline initialization (no function call overhead)
    void Init() {
        // noteParam.Init(hw.controls[1], 36.0f, 84.0f, Parameter::LINEAR);
        sequencerNotes.clear();
        sequencerNoteIndex = 0;
        sequencerNotesAscending = true;
        sequencerNoteLengthPercent = 0.5f;
        clockInterval = static_cast<uint32_t>(60000 / (clockBpm * 24));
        lastClockTime = hw.seed.system.GetNow();
        
        // Initialize subdivision timing
        currentSubdivision = 0;
        lastSubdivisionTime = 0;
        subdivisionInterval = 0; // Will be calculated after sequenceStepInterval is set
        
        InitTriggerSequence();
    }
    
    // Inline helper (no function call overhead)
    inline int GetDensityValue() {
        return static_cast<int>(density + 0.5f);
    }
    
    inline void UpdateClockInterval() {
        clockInterval = static_cast<uint32_t>(60000 / (clockBpm * 24));
    }
};

SequencerParams sequencer;

// ============================================================================
// State Management System - Centralized State Storage
// ============================================================================

// ParamId enum - identifies each user-adjustable parameter
enum ParamId {
    // ADSR Panel
    PARAM_ADSR_ATTACK = 0,
    PARAM_ADSR_DECAY_RELEASE,
    PARAM_ADSR_SUSTAIN,
    PARAM_ADSR_MIN,
    
    // MIXER Panel
    PARAM_PAN_FREQ,
    PARAM_PAN_AMP,
    PARAM_VOLUME,
    
    // OSC Panel
    PARAM_OSC_WAVEFORM,
    PARAM_OSC_MODE,
    PARAM_OSC_FM2_RATIO,
    PARAM_OSC_FM2_INDEX,
    PARAM_OSC_FORMANT_FREQ,
    PARAM_OSC_FORMANT_PHASE,
    PARAM_OSC_HARMONIC_IDX,
    PARAM_OSC_HARMONIC_DECAY,
    PARAM_OSC_HARMONIC_SKEW,
    
    // TUNING Panel
    PARAM_TUNING_INDEX,
    PARAM_TUNING_MIDI_ENABLE,
    
    // SEQUENCER Panel
    PARAM_SEQ_DENSITY,
    PARAM_SEQ_ORDER,
    PARAM_SEQ_LENGTH,
    PARAM_SEQ_BPM,
    
    // SAMPLER Panel (CC Probabilities)
    PARAM_CC_PROB_0_1,
    PARAM_CC_PROB_2_3,
    PARAM_CC_PROB_4_5,
    PARAM_CC_PROB_6_7,
    
    // SAMPLER Panel (CC Multipliers)
    PARAM_CC_MULT_0_1,
    PARAM_CC_MULT_2_3,
    PARAM_CC_MULT_4_5,
    PARAM_CC_MULT_6_7,
    
    // COMP Panel
    PARAM_COMP_RATIO,
    PARAM_COMP_THRESHOLD,
    PARAM_COMP_ATTACK,
    PARAM_COMP_RELEASE,
    
    PARAM_NONE  // Used for unbound knobs
};

// UserState struct - single source of truth for all user-adjustable parameters
// All normalizedValues stored in normalized 0.0-1.0 range
struct UserState {
    // ADSR parameters (stored directly in milliseconds)
    float adsrAttackMs;        // Attack time in milliseconds (0.1ms - 5000ms)
    float adsrDecayReleaseMs;   // Decay/Release time in milliseconds (0.1ms - 3000ms)
    float adsrSustain;          // Sustain level 0.0-1.0
    float adsrMin;              // Minimum envelope level 0.0-1.0
    
    // MIXER parameters
    float panFreq;              // 0.0-1.0 maps to 0-10Hz
    float panAmp;               // 0.0-1.0 amplitude (0=centered, 1=full pan)
    float volume;               // 0.0-1.0 master volume
    
    // OSC parameters
    float oscWaveform;          // 0.0-1.0 (sine->tri->square->saw)
    float oscMode;              // 0.0-1.0 oscillator mode (0=Interpolated, 1=FM2, 2=Formant, 3=Harmonic)
    float fm2Ratio;             // 0.0-1.0 maps to 0.125-8.0
    float fm2Index;             // 0.0-1.0 maps to 0.0-1.0
    float formantFreq;          // 0.0-1.0 maps to 10-1000 Hz
    float formantPhaseShift;   // 0.0-1.0 maps to 0.0-1.0
    float harmonicIdx;          // 0.0-1.0 maps to 1-16 as integers
    float harmonicDecay;        // 0.0-1.0 maps to 0-10 decay rate
    float harmonicSkew;         // 0.0-1.0 skew position (0=fundamental, 1=high harmonics)
    
    // TUNING parameters
    float tuningIndex;          // 0.0-1.0 maps to tuning preset index
    float tuningMidiEnable;     // 0.0-1.0 (>0.5 = enabled)
    
    // SEQUENCER parameters
    float seqDensity;           // 0.0-1.0 maps to 0-16 triggers
    float seqOrder;             // 0.0-1.0 (<0.5=ascending, >=0.5=descending)
    float seqLength;            // 0.0-1.0 note length percentage
    float seqBpm;               // 0.0-1.0 maps to CLOCK_BPM_MIN-CLOCK_BPM_MAX
    
    // SAMPLER parameters (CC probabilities)
    float ccProb0_1;            // 0.0-1.0 probability for CC slots 0 and 1
    float ccProb2_3;            // 0.0-1.0 probability for CC slots 2 and 3
    float ccProb4_5;            // 0.0-1.0 probability for CC slots 4 and 5
    float ccProb6_7;            // 0.0-1.0 probability for CC slots 6 and 7
    
    // SAMPLER parameters (CC multipliers)
    float ccMult0_1;            // 0.0-1.0 maps to multiplier 1,2,4,8 for CC slots 0 and 1
    float ccMult2_3;            // 0.0-1.0 maps to multiplier 1,2,4,8 for CC slots 2 and 3
    float ccMult4_5;            // 0.0-1.0 maps to multiplier 1,2,4,8 for CC slots 4 and 5
    float ccMult6_7;            // 0.0-1.0 maps to multiplier 1,2,4,8 for CC slots 6 and 7
    
    // COMP parameters
    float compRatio;            // 0.0-1.0 maps to 1.0-40.0 compression ratio
    float compThreshold;       // 0.0-1.0 maps to 0.0 to -80.0 dB
    float compAttack;          // 0.0-1.0 maps to 0.001-10.0 seconds
    float compRelease;         // 0.0-1.0 maps to 0.001-10.0 seconds
    
    // Constructor with default normalizedValues
    UserState() :
        adsrAttackMs(0.1f),         // 0.1ms attack
        adsrDecayReleaseMs(2500.0f), // 2.5s decay/release (2500ms)
        adsrSustain(1.0f),           // Full sustain
        adsrMin(0.0f),
        panFreq(0.02f),         // Default 0.2Hz
        panAmp(1.0f),           // Default full amplitude
        volume(0.8f),           // Default 80% volume
        oscWaveform(0.0f),      // Default sine wave
        oscMode(0.0f),          // Default Interpolated oscillator
        fm2Ratio(0.26f),        // Default 1.0 ratio (0.26 normalizes to 1.0)
        fm2Index(0.5f),         // Default 0.5 index
        formantFreq(0.5f),      // Default 500 Hz
        formantPhaseShift(0.5f), // Default 0.5 phase shift
        harmonicIdx(0.0f),      // Default harmonic index 1 (0.0 normalizes to 1)
        harmonicDecay(0.3f),    // Default decay rate
        harmonicSkew(0.0f),     // Default no skew (emphasize fundamental)
        tuningIndex(0.0f),      // Default 12-TET
        tuningMidiEnable(1.0f), // Default enabled
        seqDensity(0.0f),
        seqOrder(0.0f),         // Default ascending
        seqLength(0.5f),        // Default 50% length
        seqBpm((CLOCK_BPM_DEFAULT - CLOCK_BPM_MIN) / static_cast<float>(CLOCK_BPM_MAX - CLOCK_BPM_MIN)),  // Default 120 BPM normalized
        ccProb0_1(0.0f),
        ccProb2_3(0.0f),
        ccProb4_5(0.0f),
        ccProb6_7(0.0f),
        ccMult0_1(0.0f),        // Default multiplier 1 (0.0 maps to multiplier 1)
        ccMult2_3(0.0f),
        ccMult4_5(0.0f),
        ccMult6_7(0.0f),
        compRatio(0.25f),        // Default 4:1 ratio (0.25 = (4-1)/(40-1) ≈ 0.077)
        compThreshold(0.6f),    // Default -12 dB (0.6 in normalized range)
        compAttack(0.01f),      // Default 0.001s attack
        compRelease(0.05f)      // Default 0.1s release
    {}
};

// Global state instance
UserState appState;

// State accessor functions
float GetParamValue(ParamId paramId);
void SetParamValue(ParamId paramId, float normalizedValue);
float GetKnobValue(int panelIndex, int knobIndex);  // Helper to get knob normalizedValue from UserState

// Panel knob binding structure
struct PanelKnobBinding {
    ParamId knob1;
    ParamId knob2;
    ParamId knob3;
    ParamId knob4;
};

struct panelStruct
{
    std::string         name;
    char                id;
    std::string         input1Name;
    std::string         input2Name;
    std::string         input3Name;
    std::string         input4Name;
    float               normalizedValues[4];      // Legacy - will be removed in cleanup
    PanelKnobBinding    bindings;       // New binding system
};
panelStruct displayPanels[9] = {
    { 
        name: "ADSR",
        id: 'e',
        input1Name: "A", 
        input2Name: "D/R", 
        input3Name: "S",
        input4Name: "Min",
        normalizedValues: {0.0f, 0.0f, 0.0f, 0.0f},
        bindings: {PARAM_ADSR_ATTACK, PARAM_ADSR_DECAY_RELEASE, PARAM_ADSR_SUSTAIN, PARAM_ADSR_MIN}
    },
    {
        name: "MIXER",
        id: 'm',
        input1Name: "Freq",
        input2Name: "Amp",
        input3Name: "",
        input4Name: "Vol",
        normalizedValues: {0.0f, 0.0f, 0.0f, 0.8f},
        bindings: {PARAM_PAN_FREQ, PARAM_PAN_AMP, PARAM_NONE, PARAM_VOLUME}
    },
    {
        name: "OSC",
        id: 'o',
        input1Name: "Waveform",
        input2Name: "",
        input3Name: "",
        input4Name: "Mode",
        normalizedValues: {0.0f, 0.0f, 0.0f, 0.0f},
        bindings: {PARAM_OSC_WAVEFORM, PARAM_NONE, PARAM_NONE, PARAM_OSC_MODE}
    },
    {
        name: "TUNING",
        id: 't',
        input1Name: "T",
        input2Name: "",
        input3Name: "MIDI",
        input4Name: "",
        normalizedValues: {0.0f, 0.0f, 1.0f, 1.0f},
        bindings: {PARAM_TUNING_INDEX, PARAM_NONE, PARAM_TUNING_MIDI_ENABLE, PARAM_NONE}
    },
    {
        name: "SEQUENCER",
        id: 's',
        input1Name: "Density",
        input2Name: "Order",
        input3Name: "Length",
        input4Name: "BPM",
        normalizedValues: {0.0f, 0.0f, 0.5f, 0.0f},
        bindings: {PARAM_SEQ_DENSITY, PARAM_SEQ_ORDER, PARAM_SEQ_LENGTH, PARAM_SEQ_BPM}
    },
    {
        name: "SAMPLER",
        id: 'c',
        input1Name: "CC1-2",
        input2Name: "CC3-4",
        input3Name: "CC5-6",
        input4Name: "CC7-8",
        normalizedValues: {0.0f, 0.0f, 0.0f, 0.0f},
        bindings: {PARAM_CC_PROB_0_1, PARAM_CC_PROB_2_3, PARAM_CC_PROB_4_5, PARAM_CC_PROB_6_7}
    },
    {
        name: "MULT",
        id: 'u',
        input1Name: "M1-2",
        input2Name: "M3-4",
        input3Name: "M5-6",
        input4Name: "M7-8",
        normalizedValues: {0.0f, 0.0f, 0.0f, 0.0f},
        bindings: {PARAM_CC_MULT_0_1, PARAM_CC_MULT_2_3, PARAM_CC_MULT_4_5, PARAM_CC_MULT_6_7}
    },
    {
        name: "COMP",
        id: 'x',
        input1Name: "Ratio",
        input2Name: "Thresh",
        input3Name: "Atk",
        input4Name: "Rel",
        normalizedValues: {0.25f, 0.6f, 0.01f, 0.05f},
        bindings: {PARAM_COMP_RATIO, PARAM_COMP_THRESHOLD, PARAM_COMP_ATTACK, PARAM_COMP_RELEASE}
    },
    {
        name: "PRESET",
        id: 'p',
        input1Name: "",
        input2Name: "",
        input3Name: "",
        input4Name: "",
        normalizedValues: {0.0f, 0.0f, 0.0f, 0.0f},
        bindings: {PARAM_NONE, PARAM_NONE, PARAM_NONE, PARAM_NONE}
    }
};
int panelModesCount = sizeof(displayPanels) / sizeof(displayPanels[0]);
panelStruct currentPanel;
int noteCount = 0;

float previousKnobState [4];
float smoothedKnobState[4] = {0.0f, 0.0f, 0.0f, 0.0f};
bool knobCaughtUp[4] = {false, false, false, false};  // Track if knob has caught up to stored knob normalizedValue

// Global knob normalizedValues storage - stores all knob positions for all panels (0.0-1.0 normalized)
float knobValues[9][4] = {
    {0.0f, 0.0f, 0.0f, 0.0f},  // Panel 0: ADSR
    {0.0f, 0.0f, 0.0f, 0.8f},  // Panel 1: MIXER
    {0.0f, 0.0f, 0.0f, 0.0f},  // Panel 2: OSC
    {0.0f, 0.0f, 1.0f, 1.0f},  // Panel 3: TUNING
    {0.0f, 0.0f, 0.5f, 0.0f},  // Panel 4: SEQUENCER
    {0.0f, 0.0f, 0.0f, 0.0f},  // Panel 5: SAMPLER
    {0.0f, 0.0f, 0.0f, 0.0f},  // Panel 6: MULT 
    {0.25f, 0.6f, 0.01f, 0.05f}, // Panel 7: COMP
    {0.0f, 0.0f, 0.0f, 0.0f}   // Panel 8: PRESET
};
const float KNOB_CATCHUP_THRESHOLD = 0.05f;  // How close knob must be to catch up (5%)

struct voiceStruct
{
    int8_t note;
    int8_t velocity;
    uint32_t allocationOrder;  // Track when this voice was last allocated (for voice stealing)
    // float freq;
    // float amp;
    // float decay;
    // float damp;
    // float excite;
    // float trig;
};
voiceStruct voices[4];

struct envStruct
{
    Adsr      env;
    float     envSig;
    bool      gate;
    bool      trig;
};

// from PluckEcho example
// PolyPluck<NUM_VOICES> synth;
#define NUM_VOICES 1
// Removed pluckStruct and plucks array to save memory
// #define MAX_DELAY ((size_t)(10.0f * 48000.0f))
// 10 second delay line on the external SDRAM
// DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delay;

envStruct envelopes[4];
void      ProcessControls();
void      ApplyVCAs();
void      ApplyPanning(float* data);
void      ApplyCompression(float* data);
void      UpdateOled();
// void      plucksApply();
void      InitPan(float samplerate);
void      SendPitchBend(uint8_t channel, int16_t bendValue);
void      ProcessCCSlots();
void      ProcessCCSlotsWithSubdivision(uint8_t subdivision);
void      AddCCToQueue(uint8_t ccValue, bool isReset = false);
void      ProcessCCQueue();

// Debug message functions
void      SetDebugMessage(const char* message);
void      SetDebugMessageF(const char* format, ...);
void      ClearDebugMessage();
bool      IsDebugMessageExpired();

// SD Card functions
void      ShowPresetValues();
bool      SavePreset();
bool      LoadPreset();

// Shift Register Mode functions
void      AddNoteToQueue(int8_t note, int8_t velocity);
void      RemoveNoteFromQueue(int8_t note);
void      ClearAllVoices();

// Sequencer Mode functions
void      AddNoteToSequencer(uint8_t note);
void      RemoveNoteFromSequencer(uint8_t note);
void      SortSequencerNotes();
void      ClearSequencerNotes();
void      CaptureCurrentlyHeldNotes();
void      ApplyTuningToSequencerNotes();

bool      knobChanged = false;

// SD Card auto-save debouncing
uint32_t  lastSaveTime = 0;
const uint32_t SAVE_DEBOUNCE_MS = 2000; // Save 2 seconds after last knob change
bool      sdCardInitialized = false;

void DisplayMessage(const char* str)
{
    // Use fixed-width format to prevent overlap - avoid bottom row (y=56-63) reserved for general parameters
    // Position at y=48 to stay above bottom row
    WriteFixedString(hw, 0, 48, 11, font_s, str);  // Reduced width to 11 chars to stay left
    hw.display.Update();
}

// send simple debug string
void SetDebugMessage(const char* message)
{
    // Copy message to debug buffer (limit to DEBUG_MESSAGE_SIZE - 1 chars to leave room for null terminator)
    strncpy(debugMessage, message, DEBUG_MESSAGE_SIZE - 1);
    debugMessage[DEBUG_MESSAGE_SIZE - 1] = '\0';  // Ensure null termination
    
    // Set debug message as active and record timestamp
    debugMessageActive = true;
    debugMessageTime = hw.seed.system.GetNow();
}

// send formatted debug string
void SetDebugMessageF(const char* format, ...)
{
    // Use a temporary buffer for formatting
    char tempBuffer[DEBUG_MESSAGE_SIZE];
    
    // Format the message using va_list
    va_list args;
    va_start(args, format);
    vsnprintf(tempBuffer, DEBUG_MESSAGE_SIZE, format, args);
    va_end(args);
    
    // Ensure null termination
    tempBuffer[DEBUG_MESSAGE_SIZE - 1] = '\0';
    
    // Copy to debug buffer
    strncpy(debugMessage, tempBuffer, DEBUG_MESSAGE_SIZE - 1);
    debugMessage[DEBUG_MESSAGE_SIZE - 1] = '\0';
    
    // Set debug message as active and record timestamp
    debugMessageActive = true;
    debugMessageTime = hw.seed.system.GetNow();
}

void ClearDebugMessage()
{
    debugMessageActive = false;
    debugMessage[0] = '\0';
}

bool IsDebugMessageExpired()
{
    if (!debugMessageActive) return true;
    
    uint32_t currentTime = hw.seed.system.GetNow();
    return (currentTime - debugMessageTime) >= DEBUG_MESSAGE_DURATION_MS;
}

// ============================================================================
// State Accessor Functions Implementation
// ============================================================================

float GetParamValue(ParamId paramId)
{
    switch(paramId) {
        // ADSR Panel
        case PARAM_ADSR_ATTACK:         return appState.adsrAttackMs;
        case PARAM_ADSR_DECAY_RELEASE:  return appState.adsrDecayReleaseMs;
        case PARAM_ADSR_SUSTAIN:        return appState.adsrSustain;
        case PARAM_ADSR_MIN:            return appState.adsrMin;
        
        // MIXER Panel
        case PARAM_PAN_FREQ:            return appState.panFreq;
        case PARAM_PAN_AMP:             return appState.panAmp;
        case PARAM_VOLUME:              return appState.volume;
        
        // OSC Panel
        case PARAM_OSC_WAVEFORM:        return appState.oscWaveform;
        case PARAM_OSC_MODE:            return appState.oscMode;
        case PARAM_OSC_FM2_RATIO:       return appState.fm2Ratio;
        case PARAM_OSC_FM2_INDEX:       return appState.fm2Index;
        case PARAM_OSC_FORMANT_FREQ:    return appState.formantFreq;
        case PARAM_OSC_FORMANT_PHASE:   return appState.formantPhaseShift;
        case PARAM_OSC_HARMONIC_IDX:    return appState.harmonicIdx;
        case PARAM_OSC_HARMONIC_DECAY:  return appState.harmonicDecay;
        case PARAM_OSC_HARMONIC_SKEW:   return appState.harmonicSkew;
        
        // TUNING Panel
        case PARAM_TUNING_INDEX:        return appState.tuningIndex;
        case PARAM_TUNING_MIDI_ENABLE:  return appState.tuningMidiEnable;
        
        // SEQUENCER Panel
        case PARAM_SEQ_DENSITY:         return appState.seqDensity;
        case PARAM_SEQ_ORDER:           return appState.seqOrder;
        case PARAM_SEQ_LENGTH:          return appState.seqLength;
        case PARAM_SEQ_BPM:             return appState.seqBpm;
        
        // SAMPLER Panel (CC Probabilities)
        case PARAM_CC_PROB_0_1:         return appState.ccProb0_1;
        case PARAM_CC_PROB_2_3:         return appState.ccProb2_3;
        case PARAM_CC_PROB_4_5:         return appState.ccProb4_5;
        case PARAM_CC_PROB_6_7:         return appState.ccProb6_7;
        
        // SAMPLER Panel (CC Multipliers)
        case PARAM_CC_MULT_0_1:         return appState.ccMult0_1;
        case PARAM_CC_MULT_2_3:         return appState.ccMult2_3;
        case PARAM_CC_MULT_4_5:         return appState.ccMult4_5;
        case PARAM_CC_MULT_6_7:         return appState.ccMult6_7;
        
        // COMP Panel
        case PARAM_COMP_RATIO:         return appState.compRatio;
        case PARAM_COMP_THRESHOLD:     return appState.compThreshold;
        case PARAM_COMP_ATTACK:        return appState.compAttack;
        case PARAM_COMP_RELEASE:       return appState.compRelease;
        
        case PARAM_NONE:
        default:                        return 0.0f;
    }
}

void SetParamValue(ParamId paramId, float normalizedValue)
{
    switch(paramId) {
        // ADSR Panel
        case PARAM_ADSR_ATTACK:
            {
                // Convert normalized normalizedValue (0.0-1.0) to milliseconds
                // Use exponential mapping for better control over short times
                float attackMs = (normalizedValue * normalizedValue) * 5000.0f;
                appState.adsrAttackMs = std::max(0.1f, std::min(5000.0f, attackMs));
                
                // Apply to all envelopes (convert ms to seconds)
                for (int i = 0; i < 4; i++) {
                    float attackTime = appState.adsrAttackMs / 1000.0f; // Convert ms to seconds
                    envelopes[i].env.SetTime(ADSR_SEG_ATTACK, attackTime);
                }
            }
            break;
            
        case PARAM_ADSR_DECAY_RELEASE:
            {
                // Convert normalized normalizedValue (0.0-1.0) to milliseconds
                // Use exponential mapping for better control over short times
                float decayMs = (normalizedValue * normalizedValue) * 3000.0f;
                appState.adsrDecayReleaseMs = std::max(0.1f, std::min(3000.0f, decayMs));
                
                // Apply to all envelopes (convert ms to seconds)
                for (int i = 0; i < 4; i++) {
                    float decayTime = appState.adsrDecayReleaseMs / 1000.0f; // Convert ms to seconds
                    envelopes[i].env.SetTime(ADSR_SEG_DECAY, decayTime);
                    envelopes[i].env.SetTime(ADSR_SEG_RELEASE, decayTime);
                }
            }
            break;
            
        case PARAM_ADSR_SUSTAIN:
            // Clamp to 0.0-1.0 range
            appState.adsrSustain = std::max(0.0f, std::min(1.0f, normalizedValue));
            // Apply to all envelopes
            for (int i = 0; i < 4; i++) {
                float sustainLevel = 0.01f * powf(100.0f, normalizedValue);
                envelopes[i].env.SetSustainLevel(sustainLevel);
            }
            break;
            
        case PARAM_ADSR_MIN:
            appState.adsrMin = normalizedValue;
            voicesMinLevel = normalizedValue;
            break;
        
        // MIXER Panel
        case PARAM_PAN_FREQ:
            appState.panFreq = normalizedValue;
            panFreq = normalizedValue * 10.0f;  // Map to 0-10Hz
            break;
            
        case PARAM_PAN_AMP:
            appState.panAmp = normalizedValue;
            panAmp = normalizedValue;
            break;
            
        case PARAM_VOLUME:
            appState.volume = normalizedValue;
            // Volume is read directly in ApplyPanning
            break;
        
        // OSC Panel
        case PARAM_OSC_WAVEFORM:
            appState.oscWaveform = normalizedValue;
            // Apply to all oscillators
            for (int i = 0; i < 4; i++) {
                voiceInterpOsc[i].SetWaveformParam(normalizedValue);
            }
            break;
            
        case PARAM_OSC_MODE:
            appState.oscMode = normalizedValue;
            // Store mode for audio callback to use
            // No action needed here, the audio callback will check the mode
            break;
            
        case PARAM_OSC_FM2_RATIO:
            {
                appState.fm2Ratio = normalizedValue;
                // Map 0.0-1.0 to 0.125-8.0 (logarithmic scale for musical ratios)
                float ratio = 0.125f * powf(64.0f, normalizedValue);
                ratio = std::max(0.125f, std::min(8.0f, ratio));
                for (int i = 0; i < 4; i++) {
                    voiceFm2Osc[i].SetRatio(ratio);
                }
            }
            break;
            
        case PARAM_OSC_FM2_INDEX:
            {
                appState.fm2Index = normalizedValue;
                float index = normalizedValue; // 0.0-1.0
                for (int i = 0; i < 4; i++) {
                    voiceFm2Osc[i].SetIndex(index);
                }
            }
            break;
            
        case PARAM_OSC_FORMANT_FREQ:
            {
                appState.formantFreq = normalizedValue;
                // Map 0.0-1.0 to 10-1000 Hz (logarithmic scale)
                float freq = 10.0f * powf(100.0f, normalizedValue);
                freq = std::max(10.0f, std::min(1000.0f, freq));
                for (int i = 0; i < 4; i++) {
                    voiceFormantOsc[i].SetFormantFreq(freq);
                }
            }
            break;
            
        case PARAM_OSC_FORMANT_PHASE:
            {
                appState.formantPhaseShift = normalizedValue;
                // Phase shift 0.0-1.0
                for (int i = 0; i < 4; i++) {
                    voiceFormantOsc[i].SetPhaseShift(normalizedValue);
                }
            }
            break;
            
        case PARAM_OSC_HARMONIC_IDX:
            {
                appState.harmonicIdx = normalizedValue;
                // Map 0.0-1.0 to 1-16 (integers)
                int idx = 1 + static_cast<int>(normalizedValue * 15.99f);
                idx = std::max(1, std::min(16, idx));
                for (int i = 0; i < 4; i++) {
                    voiceHarmonicOsc[i].SetFirstHarmIdx(idx);
                    
                    // Calculate exponential decay distribution with skew
                    float decay = appState.harmonicDecay * 10.0f; // Map to 0-10
                    float skew = appState.harmonicSkew;
                    float amplitudes[16];
                    for (int h = 0; h < 16; h++) {
                        float t = h / 15.0f;
                        // Exponential decay curves
                        float lowCurve = expf(-t * decay);
                        float highCurve = expf(-(1.0f - t) * decay);
                        float weight = lowCurve * (1.0f - skew) + highCurve * skew;
                        amplitudes[h] = weight;
                    }
                    // Normalize amplitudes to prevent clipping
                    float sum = 0.0f;
                    for (int h = 0; h < 16; h++) sum += amplitudes[h];
                    if (sum > 0.0f) {
                        for (int h = 0; h < 16; h++) amplitudes[h] /= sum;
                    }
                    voiceHarmonicOsc[i].SetAmplitudes(amplitudes);
                }
            }
            break;
            
        case PARAM_OSC_HARMONIC_DECAY:
            {
                appState.harmonicDecay = normalizedValue;
                // Update distribution curve with new decay rate
                for (int i = 0; i < 4; i++) {
                    float decay = normalizedValue * 10.0f; // Map to 0-10
                    float skew = appState.harmonicSkew;
                    float amplitudes[16];
                    for (int h = 0; h < 16; h++) {
                        float t = h / 15.0f;
                        float lowCurve = expf(-t * decay);
                        float highCurve = expf(-(1.0f - t) * decay);
                        float weight = lowCurve * (1.0f - skew) + highCurve * skew;
                        amplitudes[h] = weight;
                    }
                    // Normalize
                    float sum = 0.0f;
                    for (int h = 0; h < 16; h++) sum += amplitudes[h];
                    if (sum > 0.0f) {
                        for (int h = 0; h < 16; h++) amplitudes[h] /= sum;
                    }
                    voiceHarmonicOsc[i].SetAmplitudes(amplitudes);
                }
            }
            break;
            
        case PARAM_OSC_HARMONIC_SKEW:
            {
                appState.harmonicSkew = normalizedValue;
                // Update distribution curve with new skew
                for (int i = 0; i < 4; i++) {
                    float decay = appState.harmonicDecay * 10.0f;
                    float skew = normalizedValue;
                    float amplitudes[16];
                    for (int h = 0; h < 16; h++) {
                        float t = h / 15.0f;
                        float lowCurve = expf(-t * decay);
                        float highCurve = expf(-(1.0f - t) * decay);
                        float weight = lowCurve * (1.0f - skew) + highCurve * skew;
                        amplitudes[h] = weight;
                    }
                    // Normalize
                    float sum = 0.0f;
                    for (int h = 0; h < 16; h++) sum += amplitudes[h];
                    if (sum > 0.0f) {
                        for (int h = 0; h < 16; h++) amplitudes[h] /= sum;
                    }
                    voiceHarmonicOsc[i].SetAmplitudes(amplitudes);
                }
            }
            break;
        
        // TUNING Panel
        case PARAM_TUNING_INDEX:
            {
                appState.tuningIndex = normalizedValue;
                uint8_t newTuningIndex = static_cast<uint8_t>(normalizedValue * (NUM_TUNING_PRESETS - 1) + 0.5f);
                if (newTuningIndex != currentTuningIndex) {
                    currentTuningIndex = newTuningIndex;
                    ApplyTuningToSequencerNotes();
                }
            }
            break;
            
        case PARAM_TUNING_MIDI_ENABLE:
            appState.tuningMidiEnable = normalizedValue;
            sendPitchBendMidi = (normalizedValue > 0.5f);
            break;
        
        // SEQUENCER Panel
        case PARAM_SEQ_DENSITY:
            appState.seqDensity = normalizedValue;
            sequencer.density = normalizedValue * static_cast<float>(TRIGGER_SEQUENCE_LENGTH);
            {
                int numTriggers = static_cast<int>(sequencer.density);
                numTriggers = std::max(0, std::min(numTriggers, static_cast<int>(TRIGGER_SEQUENCE_LENGTH)));
                GenerateEuclideanRhythm(numTriggers, TRIGGER_SEQUENCE_LENGTH, sequencer.triggerSequence);
            }
            break;
            
        case PARAM_SEQ_ORDER:
            {
                appState.seqOrder = normalizedValue;
                bool newAscending = normalizedValue < 0.5f;
                if (newAscending != sequencer.sequencerNotesAscending) {
                    sequencer.sequencerNotesAscending = newAscending;
                    SortSequencerNotes();
                }
            }
            break;
            
        case PARAM_SEQ_LENGTH:
            appState.seqLength = normalizedValue;
            sequencer.sequencerNoteLengthPercent = 0.1f + normalizedValue * 0.8f;  // Map to 0.1-0.9
            break;
            
        case PARAM_SEQ_BPM:
            {
                appState.seqBpm = normalizedValue;
                int32_t newBpm = CLOCK_BPM_MIN + static_cast<int32_t>(normalizedValue * (CLOCK_BPM_MAX - CLOCK_BPM_MIN));
                if (newBpm != sequencer.clockBpm) {
                    sequencer.clockBpm = newBpm;
                    sequencer.clockInterval = static_cast<uint32_t>(60000 / (sequencer.clockBpm * 24));
                    sequencer.sequenceStepInterval = static_cast<uint32_t>(60000 / (sequencer.clockBpm * 4));
                    sequencer.subdivisionInterval = sequencer.sequenceStepInterval / 8;
                }
            }
            break;
        
        // SAMPLER Panel
        case PARAM_CC_PROB_0_1:
            appState.ccProb0_1 = normalizedValue;
            {
                uint8_t prob = (normalizedValue < 0.01f) ? 0 : static_cast<uint8_t>(normalizedValue * 100.0f);
                ccSlotProbabilities[0] = prob;
                ccSlotProbabilities[1] = prob;
            }
            break;
            
        case PARAM_CC_PROB_2_3:
            appState.ccProb2_3 = normalizedValue;
            {
                uint8_t prob = (normalizedValue < 0.01f) ? 0 : static_cast<uint8_t>(normalizedValue * 100.0f);
                ccSlotProbabilities[2] = prob;
                ccSlotProbabilities[3] = prob;
            }
            break;
            
        case PARAM_CC_PROB_4_5:
            appState.ccProb4_5 = normalizedValue;
            {
                uint8_t prob = (normalizedValue < 0.01f) ? 0 : static_cast<uint8_t>(normalizedValue * 100.0f);
                ccSlotProbabilities[4] = prob;
                ccSlotProbabilities[5] = prob;
            }
            break;
            
        case PARAM_CC_PROB_6_7:
            appState.ccProb6_7 = normalizedValue;
            {
                uint8_t prob = (normalizedValue < 0.01f) ? 0 : static_cast<uint8_t>(normalizedValue * 100.0f);
                ccSlotProbabilities[6] = prob;
                ccSlotProbabilities[7] = prob;
            }
            break;
            
        // SAMPLER Panel (CC Multipliers)
        case PARAM_CC_MULT_0_1:
            appState.ccMult0_1 = normalizedValue;
            {
                // Map 0.0-1.0 to discrete multipliers: 1, 2, 4, 8
                uint8_t mult;
                if (normalizedValue < 0.25f) mult = 1;
                else if (normalizedValue < 0.5f) mult = 2;
                else if (normalizedValue < 0.75f) mult = 4;
                else mult = 8;
                ccSlotMultipliers[0] = mult;
                ccSlotMultipliers[1] = mult;
            }
            break;
            
        case PARAM_CC_MULT_2_3:
            appState.ccMult2_3 = normalizedValue;
            {
                // Map 0.0-1.0 to discrete multipliers: 1, 2, 4, 8
                uint8_t mult;
                if (normalizedValue < 0.25f) mult = 1;
                else if (normalizedValue < 0.5f) mult = 2;
                else if (normalizedValue < 0.75f) mult = 4;
                else mult = 8;
                ccSlotMultipliers[2] = mult;
                ccSlotMultipliers[3] = mult;
            }
            break;
            
        case PARAM_CC_MULT_4_5:
            appState.ccMult4_5 = normalizedValue;
            {
                // Map 0.0-1.0 to discrete multipliers: 1, 2, 4, 8
                uint8_t mult;
                if (normalizedValue < 0.25f) mult = 1;
                else if (normalizedValue < 0.5f) mult = 2;
                else if (normalizedValue < 0.75f) mult = 4;
                else mult = 8;
                ccSlotMultipliers[4] = mult;
                ccSlotMultipliers[5] = mult;
            }
            break;
            
        case PARAM_CC_MULT_6_7:
            appState.ccMult6_7 = normalizedValue;
            {
                // Map 0.0-1.0 to discrete multipliers: 1, 2, 4, 8
                uint8_t mult;
                if (normalizedValue < 0.25f) mult = 1;
                else if (normalizedValue < 0.5f) mult = 2;
                else if (normalizedValue < 0.75f) mult = 4;
                else mult = 8;
                ccSlotMultipliers[6] = mult;
                ccSlotMultipliers[7] = mult;
            }
            break;
        
        // COMP Panel
        case PARAM_COMP_RATIO:
            {
                appState.compRatio = normalizedValue;
                // Map 0.0-1.0 to 1.0-40.0 compression ratio
                float ratio = 1.0f + normalizedValue * 39.0f;
                compressor.SetRatio(ratio);
            }
            break;
            
        case PARAM_COMP_THRESHOLD:
            {
                appState.compThreshold = normalizedValue;
                // Map 0.0-1.0 to 0.0 to -80.0 dB threshold
                float threshold = -80.0f * normalizedValue;
                compressor.SetThreshold(threshold);
            }
            break;
            
        case PARAM_COMP_ATTACK:
            {
                appState.compAttack = normalizedValue;
                // Map 0.0-1.0 to 0.001-10.0 seconds with logarithmic curve
                float attack = 0.001f * powf(10000.0f, normalizedValue);
                compressor.SetAttack(attack);
            }
            break;
            
        case PARAM_COMP_RELEASE:
            {
                appState.compRelease = normalizedValue;
                // Map 0.0-1.0 to 0.001-10.0 seconds with logarithmic curve
                float release = 0.001f * powf(10000.0f, normalizedValue);
                compressor.SetRelease(release);
            }
            break;
        
        case PARAM_NONE:
        default:
            break;
    }
}

// Helper to get knob normalizedValue from UserState via panel bindings
float GetKnobValue(int panelIndex, int knobIndex)
{
    if (panelIndex < 0 || panelIndex >= 8 || knobIndex < 0 || knobIndex >= 4) {
        return 0.0f;
    }
    
    // Return the normalized knob value (0.0-1.0) directly from knobValues storage
    return knobValues[panelIndex][knobIndex];
}

void ShowPresetValues()
{
    // Calculate knob positions (same as UpdateOled)
    int knobWidth = 5;
    int knobPadding = 15;
    int knobPositions[4];
    int startX = 0;
    for (int i = 0; i < 4; i++) {
        knobPositions[i] = startX + i * (knobWidth + knobPadding);
    }
    
    // Debug: Show what normalizedValues we're displaying
    // SetDebugMessageF("v:%.1f", knobValues[0][0]);
    
    // Show the first 4 knob normalizedValues as numbers
    WriteFixedStringF(hw, knobPositions[0], 24, 6, font_s, "%.2f", knobValues[0][0]);
    WriteFixedStringF(hw, knobPositions[1], 32, 6, font_s, "%.2f", knobValues[0][1]);
    WriteFixedStringF(hw, knobPositions[2], 40, 6, font_s, "%.2f", knobValues[0][2]);
    WriteFixedStringF(hw, knobPositions[3], 48, 6, font_s, "%.2f", knobValues[0][3]);
}

void ClearPanelArea()
{
    // Clear the panel-specific area (y=24-55) to prevent overlap when switching panels
    // Reserve bottom row (y=56-63) for general parameters
    // Draw black rectangles to clear the area (false = black fill)
    hw.display.DrawRect(0, 24, 127, 55, false, true);  // Fill with black (false = black)
}

void PanEqualPowerStereo(float pan, float normalizedValue, float* left, float* right)
{
    // Equal power panning: pan goes from -1 (full left) to +1 (full right)
    // Angle goes from 0 to π/2 as pan goes from -1 to +1
    float angle = (pan + 1.0f) * M_PI * 0.25f;
    *left       = normalizedValue * cosf(angle);
    *right      = normalizedValue * sinf(angle);
}

void ApplyPanning(float* data) {
    float L1, R1, L2, R2, L3, R3, L4, R4;
    
    // Update pan phase manually to maintain control
    panPhase += 2.0f * M_PI * panFreq / hw.AudioSampleRate();
    if (panPhase >= 2.0f * M_PI) {
        panPhase -= 2.0f * M_PI;
    }
    
    // Calculate pan positions for each voice with fixed phase offsets
    // Voices are evenly distributed: 0°, 90°, 180°, 270°
    // This ensures voices 0 and 2 are opposite (hard left/right at some point in LFO cycle)
    // and voices 1 and 3 are also opposite
    // Scale by panAmp: 0 = all centered, 1 = full panning effect
    float pan0 = sinf(panPhase) * panAmp;                      // Voice 0: 0° (base phase)
    float pan1 = sinf(panPhase + M_PI * 0.5f) * panAmp;       // Voice 1: +90° = cos(phase)
    float pan2 = sinf(panPhase + M_PI) * panAmp;              // Voice 2: +180° = -sin(phase)
    float pan3 = sinf(panPhase + M_PI * 1.5f) * panAmp;       // Voice 3: +270° = -cos(phase)
    
    // Store base output for CV output
    panOutput = pan0;
    
    PanEqualPowerStereo(pan0, data[0], &L1, &R1);
    PanEqualPowerStereo(pan1, data[1], &L2, &R2);
    PanEqualPowerStereo(pan2, data[2], &L3, &R3);
    PanEqualPowerStereo(pan3, data[3], &L4, &R4);

    // Sum all voices and apply volume control from UserState
    float mixVolume = appState.volume;
    data[0] = (L1 + L2 + L3 + L4) * mixVolume;
    data[1] = (R1 + R2 + R3 + R4) * mixVolume;
    hw.seed.dac.WriteValue(DacHandle::Channel::ONE, ((panOutput + 1.0f) / 2.0f) * 4095);
}

void ApplyCompression(float* data) {
    // Compress each individual voice before mixing
    data[0] = compressor.Process(data[0]);
    data[1] = compressor.Process(data[1]);
    data[2] = compressor.Process(data[2]);
    data[3] = compressor.Process(data[3]);
}

float IncrementTowards(float normalizedValue, float target)
{
    float incrementUp = 0.01f;
    float incrementDown = 0.00001f;
    // Removed unused variable to save memory
    if (normalizedValue < target)
    {
        normalizedValue += incrementUp;
        if (normalizedValue > target)
        {
            normalizedValue = target;
        }
    }
    else if (normalizedValue > target)
    {
        normalizedValue -= incrementDown;
        if (normalizedValue < target)
        {
            normalizedValue = target;
        }
    }
    return normalizedValue;
}

// Convert MIDI note number to frequency in Hz
float MidiNoteToFrequency(int8_t note, int8_t channel)
{
    float baseFreq = 440.0f;
    if (channel == 0) {
        baseFreq = 220.0f;
    }

    if (note <= 0) return 0.0f;
    
    float frequency = baseFreq * powf(2.0f, (note - 69) / 12.0f);
    
    // Apply tuning adjustment for internal oscillators if enabled
    if (applyToInternalOsc) {
        const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
        float frequencyMultiplier = CalculateFrequencyMultiplier(note, tuning);
        frequency *= frequencyMultiplier;
    }
    
    return frequency;
}

// Process oscillator based on selected mode
float ProcessOscillator(int voiceIndex) {
    int mode = static_cast<int>(appState.oscMode * 3.99f); // 0-3
    mode = std::max(0, std::min(3, mode)); // Clamp to 0-3
    
    switch(mode) {
        case 0: // Interpolated
            return voiceInterpOsc[voiceIndex].Process();
        case 1: // FM2
            return voiceFm2Osc[voiceIndex].Process();
        case 2: // Formant
            return voiceFormantOsc[voiceIndex].Process();
        case 3: // Harmonic
            return voiceHarmonicOsc[voiceIndex].Process();
        default:
            return voiceInterpOsc[voiceIndex].Process();
    }
}

void SetOscillatorFrequency(int voiceIndex, float freq) {
    // Set frequency for all oscillator types so mode switching doesn't lose the pitch
    voiceInterpOsc[voiceIndex].SetFreq(freq);
    voiceFm2Osc[voiceIndex].SetFrequency(freq);
    voiceFormantOsc[voiceIndex].SetCarrierFreq(freq);
    voiceHarmonicOsc[voiceIndex].SetFreq(freq);
}

// Apply VCA to inputs based on envelope normalizedValues
void ApplyVCAs(float* data) {
    float envMax = 0.0f;
    float envVal = 0.0f;
    for (size_t i = 0; i < 4; i++) {
        // char message[60];
        // snprintf(message, 60, "val: %d", voices[i].note);
        // Use envelope signal directly - velocity scaling is handled by sustain level
        envVal = std::max(envelopes[i].envSig, voicesMinLevel);
        data[i] = data[i] * envVal;

        if (envVal > envMax)
        {
            envMax = envVal;
        }
    }

    int16_t outputMaxEnvelope = envMax * 4095;
    int16_t vactrolOffset = 300; // this is to bias the Intellijel vactrol
    // int16_t output = std::min(outputMaxEnvelope + vactrolOffset, static_cast<int16_t>(4095));
    hw.seed.dac.WriteValue(DacHandle::Channel::TWO, outputMaxEnvelope + vactrolOffset);
    // cvOut1 = IncrementTowards(cvOut1 + vactrolOffset, envMax);
    // hw.seed.dac.WriteValue(DacHandle::Channel::TWO, cvOut1 * 4095);
}

// due to hardware quirks, this only works with patch cables inserted into the unused inputs
// and is affected by whether the microcontroller is in plugged in
void HandleSignalDetection(AudioHandle::InputBuffer in, size_t size) {
    // Wait a bit for audio to stabilize before starting detection
    static uint32_t callbackCount = 0;
    if (!signalDetectionEnabled && callbackCount++ > 1000) {
        signalDetectionEnabled = true;  // Start detection after ~50 callbacks (audio has stabilized)
    }
    
    // Signal detection phase - accumulate RMS values during first 100ms
    // Use RMS (root mean square) to better detect sinusoidal signals
    if (signalDetectionEnabled && signalDetectionSampleCount < SIGNAL_DETECTION_SAMPLES) {
        for (size_t i = 0; i < size && signalDetectionSampleCount < SIGNAL_DETECTION_SAMPLES; i++) {
            for (int ch = 0; ch < 4; ch++) {
                float sample = in[ch][i];
                signalDetectionAccum[ch] += sample * sample;  // Accumulate squared values for RMS
            }
            signalDetectionSampleCount++;
        }
        
        // After collecting enough samples, determine which inputs have signal
        if (signalDetectionSampleCount >= SIGNAL_DETECTION_SAMPLES) {
            const float signalThreshold = 0.30f;  // RMS threshold for signal detection
            float avgSamples = static_cast<float>(SIGNAL_DETECTION_SAMPLES);

            char debugMsg[100];
            snprintf(debugMsg, 100, "RMS:%d/%d/%d/%d", 
                static_cast<int>(std::sqrt(signalDetectionAccum[0] / avgSamples) * 1000),
                static_cast<int>(std::sqrt(signalDetectionAccum[1] / avgSamples) * 1000),
                static_cast<int>(std::sqrt(signalDetectionAccum[2] / avgSamples) * 1000),
                static_cast<int>(std::sqrt(signalDetectionAccum[3] / avgSamples) * 1000));
            SetDebugMessage(debugMsg);
            
            for (int ch = 0; ch < 4; ch++) {
                // Calculate RMS: sqrt of average of squared values
                float rms = std::sqrt(signalDetectionAccum[ch] / avgSamples);
                // If signal detected, disable internal oscillator for this voice
                if (rms > signalThreshold) {
                    useInternalOscillators[ch] = false;
                }
            }
            signalDetectionEnabled = false;  // Done with detection
        }
    }
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    ProcessControls();
    
    HandleSignalDetection(in, size);
    
    // Process envelopes at audio rate for consistent timing
    for(int j = 0; j < 4; j++)
    {
        envelopes[j].envSig = envelopes[j].env.Process(envelopes[j].gate);
    }

    // float trig, nn, decay;       // Pluck Vars
    // float sig, delsig;           // Mono Audio Vars
    // synth.SetDecay(1.0);
    // Set MIDI Note for new Pluck notes.
    // nn = 24.0f + hw.GetKnobValue(DaisyPatch::CTRL_1) * 60.0f;
    // nn = 80.0f;
    // nn = static_cast<int32_t>(nn); // Quantize to semitones
    // Handle Triggering the Plucks
    // trig = 0.0f;
    // if(hw.encoder.RisingEdge() || hw.gate_input[DaisyPatch::GATE_IN_1].Trig())
    //     trig = 1.0f;

    // if (envelopes[0].gate)
    // {
    //     trig = 1.0f;
    // } else
    // {
    //     trig = 0.0f;
    // }
    

    float results[4];
    float oscOutputs[4];  // Cache oscillator outputs before processing for outputs 2 and 3

    for(size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            // results[j] = const_cast<float*>(&in[j][i]);
            results[j] = in[j][i];
        }

        // Use internal oscillators for each voice if enabled
        for (int ch = 0; ch < 4; ch++) {
            if (useInternalOscillators[ch]) {
                oscOutputs[ch] = ProcessOscillator(ch);
                results[ch] = oscOutputs[ch];
            }
        }

        // Panel 1 - Envelope/Gain
        ApplyVCAs(results);

        // Panel 2 - Compression (before panning)
        ApplyCompression(results);

        // Panel 3 - Panning and mixing
        ApplyPanning(results);

        // plucksApply(results);

        // if (currentPanel.name == "Pluck") {
        // out[0][i] = sig;
        // out[1][i] = sig;
        // }
        out[0][i] = results[0];
        out[1][i] = results[1];
        
        // Output voices 0 and 2 to audio outputs 3 and 4 when their gates are open
        // Use cached oscillator outputs (before VCA/compression/panning processing)
        if (envelopes[0].gate) {
            if (useInternalOscillators[0]) {
                out[2][i] = oscOutputs[0];
            } else {
                // Output raw input signal
                out[2][i] = in[0][i];
            }
        } else {
            out[2][i] = 0.0f;
        }
        
        if (envelopes[2].gate) {
            if (useInternalOscillators[2]) {
                out[3][i] = oscOutputs[2];
            } else {
                // Output raw input signal
                out[3][i] = in[2][i];
            }
        } else {
            out[3][i] = 0.0f;
        }
    }

    // Note: Display updates moved to main loop to prevent audio dropouts
}

void InitEnvelopes(float samplerate)
{
    for(int i = 0; i < 4; i++)
    {
        // Initialize envelope objects
        envelopes[i].env.Init(samplerate);
        
        // Initialize gate state to false (no notes playing initially)
        envelopes[i].gate = false;
    }
}

void SendMidiMesssage(uint8_t normalizedValue, uint8_t channel, const char* type)
{
    if (strcmp(type, "NOTE_ON") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + channel), normalizedValue, 127};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "NOTE_OFF") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + channel), normalizedValue, 0};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "TRIGGER_ON") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + channel), normalizedValue, 127};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "TRIGGER_OFF") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + channel), normalizedValue, 0};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "CC") == 0)
    {
        uint8_t controller = 3; // this is configured in Intellijel 1U
        uint8_t bytes[3] = {static_cast<uint8_t>(0xB0 + channel), controller, normalizedValue};
        hw.midi.SendMessage(bytes, 3);
        
        // Debug: Show CC message being sent
        // SetDebugMessageF("Send CC:%d Ch:%d", normalizedValue, channel);
    }
}

void SendMidiClock()
{
    // Send MIDI Clock message (0xF8)
    uint8_t clockByte = 0xF8;
    hw.midi.SendMessage(&clockByte, 1);
}

void SendPitchBend(uint8_t channel, int16_t bendValue)
{
    // Send MIDI Pitch Bend message
    // Format: 0xE0 + channel, LSB, MSB
    uint8_t lsb = bendValue & 0x7F;        // Lower 7 bits
    uint8_t msb = (bendValue >> 7) & 0x7F;  // Upper 7 bits
    uint8_t bytes[3] = {static_cast<uint8_t>(0xE0 + channel), lsb, msb};
    
    hw.midi.SendMessage(bytes, 3);
    
    // Store current pitch bend normalizedValue for this channel
    currentPitchBendValues[channel] = bendValue;
}

int8_t getCurrentHighestNote() {
    int8_t highestNote = voices[0].note;
    for (int i = 1; i < 4; i++)
    {
        highestNote = std::max(highestNote, voices[i].note);
    }
    return highestNote;
}

int8_t getCurrentLowestNote() {
    int8_t lowestNote = 127;
    for (int i = 0; i < 4; i++)
    {
        if (voices[i].note == 0) continue;
        lowestNote = std::min(lowestNote, voices[i].note);
    }
    return lowestNote;
}

void HandleMidiMessage(MidiEvent m)
{   
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent event = m.AsNoteOn();
            ProcessHandlerChainNoteOn(event);
            break;
        }
        case NoteOff:
        {
            NoteOffEvent event = m.AsNoteOff();
            ProcessHandlerChainNoteOff(event);
            break;
        }
        default: break;
    }
}

int main(void)
{
    float samplerate;
    size_t blocksize = 8;
    hw.Init();
    
    samplerate = hw.AudioSampleRate();
    
    // Initialize envelopes FIRST (before loading settings)
    // This ensures the envelope objects exist before SetParamValue tries to configure them
    InitEnvelopes(samplerate);

    // Initialize SD Card
    SdmmcHandler::Config sd_cfg;
    sd_cfg.Defaults();
    SdmmcHandler::Result sd_result = sdcard.Init(sd_cfg);
    
    const char* initMessage = "init failed";
    if (sd_result == SdmmcHandler::Result::OK) {
        uint8_t bsp_result = BSP_SD_Init();
        if (bsp_result == MSD_OK) {
            sdCardInitialized = true;
            initMessage = LoadPreset() ? "preset loaded" : "load failed";
        }
    }
    SetDebugMessage(initMessage);

    // Apply loaded normalizedValues to all parameters
    for (int panel = 0; panel < panelModesCount; panel++) {
        const PanelKnobBinding& binding = displayPanels[panel].bindings;
        ParamId params[4] = {binding.knob1, binding.knob2, binding.knob3, binding.knob4};
        
        for (int knob = 0; knob < 4; knob++) {
            if (params[knob] != PARAM_NONE) {
                SetParamValue(params[knob], knobValues[panel][knob]);
            }
        }
    }
    
    // Initialize pitch bend normalizedValues to center (no bend)
    for (int i = 0; i < 16; i++) {
        currentPitchBendValues[i] = 8192;
    }

    panelMode = 0;
    currentPanel = displayPanels[panelMode];
    
    // Initialize catch-up state for the initial panel
    // Unbound knobs (PARAM_NONE) should be marked as caught up
    const PanelKnobBinding& initialBinding = displayPanels[panelMode].bindings;
    ParamId initialKnobParams[4] = {initialBinding.knob1, initialBinding.knob2, initialBinding.knob3, initialBinding.knob4};
    for (int i = 0; i < 4; i++) {
        knobCaughtUp[i] = (initialKnobParams[i] == PARAM_NONE);
    }
    
    // Initialize sequencer parameters
    sequencer.Init();
    
    // Initialize BPM state normalizedValue based on current BPM
    // Map BPM to 0-1 range for state storage
    float bpmKnobValue = static_cast<float>(sequencer.clockBpm - CLOCK_BPM_MIN) / (CLOCK_BPM_MAX - CLOCK_BPM_MIN);
    appState.seqBpm = bpmKnobValue;

    UpdateOled();

    // start MIDI handler
    hw.midi.StartReceive();

    for (int i = 0; i < 4; i++)
    {
        voices[i].note = 60;
        voices[i].velocity = 0;
        voices[i].allocationOrder = 0;

        // pluck init
        // plucks[i].decay = 1.0;
        // plucks[i].wetDry = 0.5;
        // plucks[i].synth.SetDecay(1.0);
        // plucks[i].synth.Init(samplerate);

    }
    // synth.Init(samplerate);

    // 
    InitPan(samplerate);
    
    // Initialize compressor
    compressor.Init(samplerate);
    compressor.SetRatio(4.0f);           // Default 4:1 ratio
    compressor.SetThreshold(-12.0f);     // Default -12 dB
    compressor.SetAttack(0.003f);        // Default 3ms attack
    compressor.SetRelease(0.1f);         // Default 100ms release
    compressor.AutoMakeup(true);         // Enable auto makeup gain
    
    // Initialize interpolated oscillators for all 4 voices
    for (int i = 0; i < 4; i++) {
        voiceInterpOsc[i].Init(samplerate);
        voiceInterpOsc[i].SetFreq(440.0f);
        voiceInterpOsc[i].SetAmp(1.0f);
        voiceInterpOsc[i].SetWaveformParam(0.0f);  // Start with sine wave
    }
    
    // Initialize FM2 oscillators
    for (int i = 0; i < 4; i++) {
        voiceFm2Osc[i].Init(samplerate);
        voiceFm2Osc[i].SetFrequency(440.0f);
        voiceFm2Osc[i].SetRatio(1.0f);
        voiceFm2Osc[i].SetIndex(1.0f);
    }
    
    // Initialize Formant oscillators
    for (int i = 0; i < 4; i++) {
        voiceFormantOsc[i].Init(samplerate);
        voiceFormantOsc[i].SetCarrierFreq(440.0f);
        voiceFormantOsc[i].SetFormantFreq(2000.0f);
        voiceFormantOsc[i].SetPhaseShift(0.5f);
    }
    
    // Initialize Harmonic oscillators
    for (int i = 0; i < 4; i++) {
        voiceHarmonicOsc[i].Init(samplerate);
        voiceHarmonicOsc[i].SetFreq(440.0f);
        voiceHarmonicOsc[i].SetFirstHarmIdx(1);
        float amplitudes[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        voiceHarmonicOsc[i].SetAmplitudes(amplitudes);
    }
    
    // Initialize OSC parameters with sensible defaults
    SetParamValue(PARAM_OSC_WAVEFORM, 0.0f);        // Start with sine
    SetParamValue(PARAM_OSC_FM2_RATIO, 0.26f);       // 1:1 ratio
    SetParamValue(PARAM_OSC_FM2_INDEX, 0.5f);        // Medium index
    SetParamValue(PARAM_OSC_FORMANT_FREQ, 0.5f);     // 500Hz
    SetParamValue(PARAM_OSC_FORMANT_PHASE, 0.5f);   // Center phase
    SetParamValue(PARAM_OSC_HARMONIC_IDX, 0.0f);    // First harmonic
    SetParamValue(PARAM_OSC_HARMONIC_DECAY, 0.3f);  // Decay rate
    SetParamValue(PARAM_OSC_HARMONIC_SKEW, 0.0f);   // No skew (fundamental emphasis)

    // Initialize signal detection - reset accumulators but don't enable yet
    for (int i = 0; i < 4; i++) {
        signalDetectionAccum[i] = 0.0f;
        signalDetectionPeak[i] = 0.0f;
    }
    signalDetectionSampleCount = 0;
    signalDetectionEnabled = false;  // Will be enabled after audio stabilizes
    
    // Start the ADC and Audio Peripherals on the Hardware
    hw.StartAdc();
    hw.SetAudioBlockSize(blocksize);
    hw.StartAudio(AudioCallback);

    for(;;)
    {
        // Handle MIDI events
        hw.midi.Listen();
        while(hw.midi.HasEvents())
        {
            HandleMidiMessage(hw.midi.PopEvent());
        }

        // Throttled display updates to prevent audio dropouts
        uint32_t currentTime = hw.seed.system.GetNow();
        if (knobChanged || (currentTime - lastDisplayUpdate) >= DISPLAY_UPDATE_INTERVAL_MS)
        {
            UpdateOled();
            knobChanged = false;
            lastDisplayUpdate = currentTime;
        }
        
        // // Check for trigger off timing
        // if (triggerOffPending && currentTime >= triggerOffTime)
        // {
        //     // Send trigger off on channel 15 (matches the trigger on sent earlier)
        //     SendMidiMesssage(sequencer.triggerNote, sequencer.ccTriggerChannel, "TRIGGER_OFF");
        //     triggerOffPending = false;
        // }
        
        // Check for CC-triggered trigger off timing
        if (ccTriggerOffPending && currentTime >= ccTriggerOffTime)
        {
            // Send CC-triggered trigger off on channel 15
            // SendMidiMesssage(sequencer.triggerNote, sequencer.ccTriggerChannel, "TRIGGER_OFF");
            ccTriggerOffPending = false;
        }
        
        // Process CC queue
        ProcessCCQueue();
        
        // Check for sequence trigger off timing
        // if (sequenceTriggerOffPending && currentTime >= sequenceTriggerOffTime)
        // {
        //     SendMidiMesssage(triggerNote, ccTriggerChannel, "TRIGGER_OFF");
        //     sequenceTriggerOffPending = false;
        // }
        
        // Check for sequencer note-off timing
        if (sequencer.sequencerNoteOffPending && currentTime >= sequencer.sequencerNoteOffTime)
        {
            if (shiftRegisterMode) {
                // Send note-off to shift register system
                RemoveNoteFromQueue(static_cast<int8_t>(sequencer.sequencerNoteToTurnOff));
            } else if (sequencer.sequencerVoiceToTurnOff >= 0) {
                // Send MIDI note-off to external devices (when shift register mode is off)
                uint8_t bytes[3] = {
                    static_cast<uint8_t>(0x80 + sequencer.sequencerVoiceToTurnOff), 
                    sequencer.sequencerNoteToTurnOff, 
                    0
                };
                hw.midi.SendMessage(bytes, 3);
                
                // Turn off the envelope gate
                envelopes[sequencer.sequencerVoiceToTurnOff].gate = false;
                
                // Clear voice data
                voices[sequencer.sequencerVoiceToTurnOff].note = 0;
                voices[sequencer.sequencerVoiceToTurnOff].velocity = 0;
            }
            sequencer.sequencerNoteOffPending = false;
        }

        // Check for MIDI clock timing
        if (sequencer.clockEnabled && (currentTime - sequencer.lastClockTime) >= sequencer.clockInterval)
        {
            SendMidiClock();
            sequencer.lastClockTime = currentTime;
        }

        // Update trigger sequence
        ProcessSequencerMidiSource();
    }
}

// Helper function to format parameter normalizedValues based on panel context
std::string FormatParameterValue(char panelId, int paramIndex, float normalizedValue)
{
    if (panelId == 'e') {
        switch(paramIndex) {
            case 0: // Attack time - convert normalized value to milliseconds
                {
                    float attackMs = 0.1f + (normalizedValue) * 4999.9f;
                    return std::to_string(static_cast<int>(attackMs)) + "ms";
                }
            case 1: // Decay/Release time - convert normalized value to milliseconds
                {
                    float decayMs = 0.1f + (normalizedValue) * 4999.9f;
                    return std::to_string(static_cast<int>(decayMs)) + "ms";
                }
            case 2: // Sustain level
                return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
            case 3: // Minimum level
                return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
            default: return "0%";
        }
    }
    else if (panelId == 'm') {
        switch(paramIndex) {
            case 0: // Frequency
                return std::to_string(static_cast<int>(normalizedValue * 10)) + "Hz";
            case 1: // Amplitude
                return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
            case 3: // Volume
                return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
            default: return "0%";
        }
    }
    else if (panelId == 'o') {
        switch(paramIndex) {
            case 0: // Knob 1 - Mode specific
                {
                    int mode = static_cast<int>(appState.oscMode * 3.99f);
                    mode = std::max(0, std::min(3, mode));
                    switch(mode) {
                        case 0: // Interpolated - waveform
                            if (normalizedValue < 0.25f) return "Sine";
                            else if (normalizedValue < 0.5f) return "Tri";
                            else if (normalizedValue < 0.75f) return "Sqr";
                            else return "Saw";
                        case 1: // FM2 - ratio
                            {
                                float ratio = 0.125f * powf(64.0f, normalizedValue);
                                ratio = std::max(0.125f, std::min(8.0f, ratio));
                                return std::to_string(static_cast<int>(ratio * 100.0f)) + "%";
                            }
                        case 2: // Formant - frequency
                            {
                                float freq = 10.0f * powf(100.0f, normalizedValue);
                                freq = std::max(10.0f, std::min(1000.0f, freq));
                                return std::to_string(static_cast<int>(freq)) + "Hz";
                            }
                        case 3: // Harmonic - idx
                            {
                                int idx = 1 + static_cast<int>(normalizedValue * 15.99f);
                                idx = std::max(1, std::min(16, idx));
                                return "H" + std::to_string(idx);
                            }
                        default:
                            return "Sine";
                    }
                }
            case 1: // Knob 2 - Mode specific
                {
                    int mode = static_cast<int>(appState.oscMode * 3.99f);
                    mode = std::max(0, std::min(3, mode));
                    switch(mode) {
                        case 0: // Interpolated - empty
                            return "";
                        case 1: // FM2 - index
                            return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
                        case 2: // Formant - phase
                            return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
                        case 3: // Harmonic - decay
                            {
                                float decay = normalizedValue * 10.0f;
                                return std::to_string(static_cast<int>(decay * 10.0f)) + "x";
                            }
                        default:
                            return "";
                    }
                }
            case 2: // Knob 3 - Mode specific
                {
                    int mode = static_cast<int>(appState.oscMode * 3.99f);
                    mode = std::max(0, std::min(3, mode));
                    switch(mode) {
                        case 0: // Interpolated - empty
                        case 1: // FM2 - empty
                        case 2: // Formant - empty
                            return "";
                        case 3: // Harmonic - skew
                            return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
                        default:
                            return "";
                    }
                }
            case 3: // Oscillator Mode
                {
                    int mode = static_cast<int>(normalizedValue * 3.99f); // 0-3
                    mode = std::max(0, std::min(3, mode));
                    switch(mode) {
                        case 0: return "Inter";
                        case 1: return "FM2";
                        case 2: return "Form";
                        case 3: return "Harm";
                        default: return "Inter";
                    }
                }
            default: return "Inter";
        }
    }
    else if (panelId == 's') {
        switch(paramIndex) {
            case 0: // Density
                return std::to_string(static_cast<int>(normalizedValue * 16)) + "/16";
            case 1: // Order
                return normalizedValue < 0.5f ? "ASC" : "DESC";
            case 2: // Length
                return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
            case 3: // BPM
                {
                    // Map 0-1 to CLOCK_BPM_MIN-CLOCK_BPM_MAX
                    int bpm = CLOCK_BPM_MIN + static_cast<int>(normalizedValue * (CLOCK_BPM_MAX - CLOCK_BPM_MIN));
                    return std::to_string(bpm);
                }
            default: return "0";
        }
    }
    else if (panelId == 't') {
        switch(paramIndex) {
            case 0: // Tuning selector
                return std::to_string(static_cast<int>(normalizedValue * 9)) + "/9";
            case 2: // MIDI
                return normalizedValue > 0.5f ? "ON" : "OFF";
            default: return "OFF";
        }
    }
    else if (panelId == 'c') {
        switch(paramIndex) {
            case 0: // CC1-2
            case 1: // CC3-4
            case 2: // CC5-6
            case 3: // CC7-8
                {
                    uint8_t probability = (normalizedValue < 0.01f) ? 0 : static_cast<uint8_t>(normalizedValue * 100.0f);
                    return (probability == 0) ? "OFF" : std::to_string(probability) + "%";
                }
            default: return "OFF";
        }
    }
    else if (panelId == 'u') {
        switch(paramIndex) {
            case 0: // Multiplier for CC1-2
            case 1: // Multiplier for CC3-4
            case 2: // Multiplier for CC5-6
            case 3: // Multiplier for CC7-8
                {
                    // Map normalized 0.0-1.0 to discrete multipliers: 1, 2, 4, 8
                    uint8_t mult;
                    if (normalizedValue < 0.25f) mult = 1;
                    else if (normalizedValue < 0.5f) mult = 2;
                    else if (normalizedValue < 0.75f) mult = 4;
                    else mult = 8;
                    return "x" + std::to_string(mult);
                }
            default: return "x1";
        }
    }
    
    // Default fallback
    return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
}

// Function to calculate knob positions dynamically, aligned to right edge
void CalculateKnobPositions(int knobWidth, int knobPadding, int knobPositions[4]) {
    const int displayWidth = 128;
    const int numKnobs = 4;
    
    // Calculate total width needed for all knobs and padding
    int totalKnobWidth = numKnobs * knobWidth + (numKnobs - 1) * knobPadding;
    
    // Calculate startX to align everything to the right edge
    int startX = displayWidth - totalKnobWidth;
    
    // Calculate individual knob positions
    for (int i = 0; i < numKnobs; i++) {
        knobPositions[i] = startX + i * (knobWidth + knobPadding);
    }
}

// Get dynamic OSC panel label names based on current mode
void GetOscLabels(int mode, std::string& label1, std::string& label2, std::string& label3) {
    switch(mode) {
        case 0: // Interpolated
            label1 = "Wave";
            label2 = "";
            label3 = "";
            break;
        case 1: // FM2
            label1 = "Ratio";
            label2 = "Index";
            label3 = "";
            break;
        case 2: // Formant
            label1 = "Freq";
            label2 = "Phase";
            label3 = "";
            break;
        case 3: // Harmonic
            label1 = "Idx";
            label2 = "Decay";
            label3 = "Skew";
            break;
        default:
            label1 = "Wave";
            label2 = "";
            label3 = "";
            break;
    }
}

void UpdateOled()
{
    // Clear the panel-specific area to prevent overlap when switching panels
    // Reserve bottom row (y=56-63) for general parameters
    ClearPanelArea();
    
    // Draw vertical bar background
    hw.display.DrawRect(0, 0, 15, 63, false, true);  // Black background
    
    // Display panel name vertically (one character per row)
    int startY = 0;
    for (size_t i = 0; i < currentPanel.name.length() && i < 7; i++) {
        char charStr[2] = {currentPanel.name[i], '\0'};
        hw.display.SetCursor(2, startY + i * 9);  // 9 pixels between rows for font_s
        hw.display.WriteString(charStr, font_s, true);
    }
    
    // Define layout parameters
    int knobWidth = 25;
    int knobPadding = 5;
    int paramValueY = 12;  // Y position for parameter normalizedValue labels
    
    int knobPositions[4];
    CalculateKnobPositions(knobWidth, knobPadding, knobPositions);
    
    // knob input labels at the top
    int labelY = 0;  // Position at very top
    
    // For OSC panel, use dynamic labels and values based on mode
    bool isOscPanel = (currentPanel.id == 'o');
    int oscMode = 0;
    
    if (isOscPanel) {
        oscMode = static_cast<int>(appState.oscMode * 3.99f);
        oscMode = std::max(0, std::min(3, oscMode));
        std::string label1, label2, label3;
        GetOscLabels(oscMode, label1, label2, label3);
        WriteFixedString(hw, knobPositions[0], labelY, 5, font_s, label1.c_str());
        WriteFixedString(hw, knobPositions[1], labelY, 5, font_s, label2.c_str());
        WriteFixedString(hw, knobPositions[2], labelY, 5, font_s, label3.c_str());
        WriteFixedString(hw, knobPositions[3], labelY, 5, font_s, currentPanel.input4Name.c_str());
    } else {
        WriteFixedString(hw, knobPositions[0], labelY, 5, font_s, currentPanel.input1Name.c_str());
        WriteFixedString(hw, knobPositions[1], labelY, 5, font_s, currentPanel.input2Name.c_str());
        WriteFixedString(hw, knobPositions[2], labelY, 5, font_s, currentPanel.input3Name.c_str());
        WriteFixedString(hw, knobPositions[3], labelY, 5, font_s, currentPanel.input4Name.c_str());
    }
    
    // Draw horizontal meters below labels
    int meterY = 8;  // Position below labels
    int maxMeterWidth = 22;  // Maximum meter width in pixels (fits within 25 pixel knob width)

    for (int i = 0; i < 4; i++)
    {
        // Clear the meter area first (draw black line to erase previous meter)
        hw.display.DrawLine(knobPositions[i], meterY, knobPositions[i] + maxMeterWidth, meterY, false);
        
        // For OSC panel, get the appropriate parameter value based on mode
        float val = GetKnobValue(panelMode, i);
        if (isOscPanel && i < 3) {
            // Get parameter value based on mode
            switch(oscMode) {
                case 0: // Interpolated
                    if (i == 0) val = appState.oscWaveform;
                    break;
                case 1: // FM2
                    if (i == 0) val = appState.fm2Ratio;
                    else if (i == 1) val = appState.fm2Index;
                    break;
                case 2: // Formant
                    if (i == 0) val = appState.formantFreq;
                    else if (i == 1) val = appState.formantPhaseShift;
                    break;
                case 3: // Harmonic
                    if (i == 0) val = appState.harmonicIdx;
                    else if (i == 1) val = appState.harmonicDecay;
                    else if (i == 2) val = appState.harmonicSkew;
                    break;
            }
        }
        int meterWidth = static_cast<int>(val * maxMeterWidth);  // Scale 0.0-1.0 to 0-22 pixels
        meterWidth = std::max(0, std::min(meterWidth, maxMeterWidth));  // Clamp to 0-22 range
        
        if (meterWidth > 0) {
            // If knob hasn't caught up, draw a dotted line to show target normalizedValue
            if (!knobCaughtUp[i]) {
                // Draw dotted line for target normalizedValue (every other pixel)
                for (int x = 0; x < meterWidth; x += 2) {
                    hw.display.DrawPixel(knobPositions[i] + x, meterY, true);
                }
            } else {
                // Draw solid line for active/caught-up knobs
                hw.display.DrawLine(knobPositions[i], meterY, knobPositions[i] + meterWidth, meterY, true);
            }
        }
    }
    
    // Display parameter normalizedValues below meters
    for (int i = 0; i < 4; i++) {
        float paramValue = GetKnobValue(panelMode, i);
        
        // For OSC panel, override with mode-specific parameters for display
        if (isOscPanel && i < 3) {
            switch(oscMode) {
                case 0: // Interpolated
                    if (i == 0) paramValue = appState.oscWaveform;
                    break;
                case 1: // FM2
                    if (i == 0) paramValue = appState.fm2Ratio;
                    else if (i == 1) paramValue = appState.fm2Index;
                    break;
                case 2: // Formant
                    if (i == 0) paramValue = appState.formantFreq;
                    else if (i == 1) paramValue = appState.formantPhaseShift;
                    break;
                case 3: // Harmonic
                    if (i == 0) paramValue = appState.harmonicIdx;
                    else if (i == 1) paramValue = appState.harmonicDecay;
                    else if (i == 2) paramValue = appState.harmonicSkew;
                    break;
            }
        }
        
        std::string paramValueStr = FormatParameterValue(currentPanel.id, i, paramValue);
        WriteFixedString(hw, knobPositions[i], paramValueY, 5, font_s, paramValueStr.c_str());
    }
    
    // Show trigger sequence pattern when in TRIGSEQ mode
    if (currentPanel.id == 's') {
        // Show step counter with fixed width - move to avoid bottom-right area
        WriteFixedStringF(hw, knobPositions[0], 24, 8, font_s, "Step:%02d", sequencer.currentSequenceStep);

        // Show density normalizedValue with fixed width - move to avoid bottom-right area
        int numTriggers = 0;
        for (int i = 0; i < TRIGGER_SEQUENCE_LENGTH; i++) {
            if (sequencer.triggerSequence[i]) numTriggers++;
        }
        WriteFixedStringF(hw, knobPositions[2], 24, 5, font_s, "D:%02d", numTriggers);
        
        // Show sequence pattern as dots in one fixed-width string
        char patternStr[TRIGGER_SEQUENCE_LENGTH + 1];
        for (int i = 0; i < TRIGGER_SEQUENCE_LENGTH; i++) {
            patternStr[i] = sequencer.triggerSequence[i] ? '*' : '-';
        }
        patternStr[TRIGGER_SEQUENCE_LENGTH] = '\0';
        WriteFixedString(hw, knobPositions[0], 32, TRIGGER_SEQUENCE_LENGTH, font_s, patternStr);
        
        // Show sequencer-specific information
        if (sequencer.sequencerMode) {
            // Show note ordering direction
            WriteFixedString(hw, knobPositions[3], 32, 4, font_s, sequencer.sequencerNotesAscending ? "ASC" : "DESC");
            
            // Show number of held notes
            WriteFixedStringF(hw, knobPositions[0], 40, 4, font_s, "N:%d", static_cast<int>(sequencer.sequencerNotes.size()));
        }
    }
    // Show tuning information when in TUNING mode
    else if (currentPanel.id == 't') {
        // Show current tuning name with fixed width (21 chars to fill display width)
        const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
        WriteFixedString(hw, knobPositions[0], 24, 21, font_s, tuning->name);
    }
    // Show CC Slots information when in CC SLOTS mode
    else if (currentPanel.id == 'c') {
        // Show current probabilities for each slot pair
        const char* slotLabels[4] = {"CC1-2", "CC3-4", "CC5-6", "CC7-8"};
        for (int i = 0; i < 4; i++) {
            float normalizedValue = GetKnobValue(5, i);  // Panel 5 = SAMPLER
            uint8_t probability = (normalizedValue < 0.01f) ? 0 : static_cast<uint8_t>(normalizedValue * 100.0f);
            std::string display = (probability == 0) ? "OFF" : std::to_string(probability) + "%";
            WriteFixedStringF(hw, knobPositions[i], 24, 6, font_s, "%s:%s", slotLabels[i], display.c_str());
        }
        
        // Show global note counter and queue status - move to avoid bottom-right area
        // WriteFixedStringF(hw, knobPositions[0], 32, 8, font_s, "Note:%d", globalNoteCounter);
        // WriteFixedStringF(hw, knobPositions[2], 32, 4, font_s, "Q:%d", ccQueueCount);
    }
    // PRESET panel - show first 4 knob normalizedValues
    else if (currentPanel.id == 'p') {
        ShowPresetValues();
    }
    
    // === BOTTOM ROW: General State Info (always visible) ===
    // Use entire bottom row (y=56-63) for general parameters
    
    // Check if debug message should be displayed
    if (debugMessageActive && !IsDebugMessageExpired()) {
        // Display debug message across the entire bottom row
        WriteFixedString(hw, knobPositions[0], 56, 31, font_s, debugMessage);
        
        // Clear debug message if it has expired
        if (IsDebugMessageExpired()) {
            ClearDebugMessage();
        }
    } else {
        // Display current note and BPM in compact format: "60|120" - left side
        WriteFixedStringF(hw, knobPositions[0], 56, 10, font_s, "%3d|%3d", currentNote, sequencer.clockBpm);

        // Display mode indicators - right side with proper spacing
        // if (sequencerMode && shiftRegisterMode) {
            // Both modes active
            // WriteFixedString(hw, knobPositions[3], 56, 4, font_s, "SQ+SR");
        // } else if (sequencerMode) {
        if (sequencer.sequencerMode) {
            // Sequencer mode active only
            WriteFixedString(hw, 100, 56, 4, font_s, "SQ");
        } else {
            WriteFixedString(hw, 100, 56, 5, font_s, "  ");
        }
        
        if (shiftRegisterMode) {
            // Shift register mode active only
            WriteFixedString(hw, 116, 56, 4, font_s, "SR");
        } else {
            // No special mode active - clear the area
            WriteFixedString(hw, 116, 56, 5, font_s, "  ");
        }
    }
    
    hw.display.Update();
}

void ProcessEncoder()
{
    bool encoderPressed = hw.encoder.Pressed();
    float timeHeld = hw.encoder.TimeHeldMs();
    
    // Check for encoder rotation (clockwise/counterclockwise)
    int32_t encoderValue = hw.encoder.Increment();
    if (encoderValue != 0) {
        // Navigate panels: clockwise = next panel, counterclockwise = previous panel
        if (encoderValue > 0) {
            // Clockwise rotation - next panel
            panelMode = (panelMode + 1) % panelModesCount;
        } else {
            // Counterclockwise rotation - previous panel
            panelMode = (panelMode - 1 + panelModesCount) % panelModesCount;
        }
        
        currentPanel = displayPanels[panelMode];
        
        // Reset knob catch-up state when switching panels
        // This prevents parameters from jumping when knobs are at different positions
        const PanelKnobBinding& binding = displayPanels[panelMode].bindings;
        ParamId knobParams[4] = {binding.knob1, binding.knob2, binding.knob3, binding.knob4};
        
        for (int i = 0; i < 4; i++) {
            // If knob is unbound (PARAM_NONE), mark as caught up immediately
            knobCaughtUp[i] = (knobParams[i] == PARAM_NONE);
        }
        
        knobChanged = true; // Trigger display update
    }

    // Detect long press: trigger when held >= 0.5 seconds
    if(encoderPressed && timeHeld >= ENCODER_SEQUENCER_TOGGLE_MS && !longPressHandled)
    {
        // Long press detected - toggle shift register mode
        shiftRegisterMode = !shiftRegisterMode;
        longPressHandled = true;
        
        if (shiftRegisterMode) {
            // SetDebugMessage("Shift Register ON");
        } else {
            // SetDebugMessage("Shift Register OFF");
            // Clear all voices when disabling shift register mode
            ClearAllVoices();
            ResetCCState(); // Reset CC state when disabling shift register
        }
        
        // Update display to show mode change
        UpdateOled();
    }
    else if(!encoderPressed && encoderWasPressed)
    {
        // Button was released
        if(!longPressHandled)
        {
            // Check if we're in PRESET panel
            if (currentPanel.id == 'p') {
                // In PRESET panel - save all parameters
                if (sdCardInitialized) {
                    SavePreset();
                } else {
                    SetDebugMessage("SD not init");
                }
                UpdateOled();
            } else {
                // Not in PRESET panel - toggle sequencer mode as normal
                sequencer.sequencerMode = !sequencer.sequencerMode;
                
                if (sequencer.sequencerMode) {
                    // When enabling sequencer mode, capture currently held notes
                    CaptureCurrentlyHeldNotes();
                    } else {
                        // Clear sequencer notes when disabling sequencer mode to prevent artifacts
                        ClearSequencerNotes();
                        ResetCCState(); // Reset CC state when disabling sequencer
                    }
                
                UpdateOled();
            }
        }
        
        // Reset long press flags for next press
        longPressHandled = false;
    }
    
    encoderWasPressed = encoderPressed;
}

void ProcessKnobs()
{
    float inputs[4];
    int8_t inputIndex = -1; // assuming only one knob changes at a time
    float knobThreshold = 0.0001; // lower normalizedValues for slower movement
    float alpha = 0.25; // higher normalizedValue for less smoothing
    float knobMax = 0.96; // knobs don't seem to go above this normalizedValue

    for (int i = 0; i < 4; i++)
    {
        inputs[i] = hw.controls[i].Process() / knobMax;
        
        // Apply exponential moving average filter
        smoothedKnobState[i] = (alpha * inputs[i]) + ((1.0f - alpha) * smoothedKnobState[i]);
        
        // Compare smoothed normalizedValues against threshold
        if (fabs(smoothedKnobState[i] - previousKnobState[i]) > knobThreshold)
        {
            inputIndex = i;
        }
    }

    if (inputIndex > -1) {
        // Get the parameter ID for this knob
        const PanelKnobBinding& binding = displayPanels[panelMode].bindings;
        ParamId paramId = PARAM_NONE;
        switch(inputIndex) {
            case 0: paramId = binding.knob1; break;
            case 1: paramId = binding.knob2; break;
            case 2: paramId = binding.knob3; break;
            case 3: paramId = binding.knob4; break;
        }
        
        // For OSC panel, route knobs to mode-specific parameters
        if (currentPanel.id == 'o' && inputIndex < 3) {
            int mode = static_cast<int>(appState.oscMode * 3.99f);
            mode = std::max(0, std::min(3, mode));
            
            ParamId modeSpecificParam = PARAM_NONE;
            switch(mode) {
                case 0: // Interpolated
                    if (inputIndex == 0) modeSpecificParam = PARAM_OSC_WAVEFORM;
                    break;
                case 1: // FM2
                    if (inputIndex == 0) modeSpecificParam = PARAM_OSC_FM2_RATIO;
                    else if (inputIndex == 1) modeSpecificParam = PARAM_OSC_FM2_INDEX;
                    break;
                case 2: // Formant
                    if (inputIndex == 0) modeSpecificParam = PARAM_OSC_FORMANT_FREQ;
                    else if (inputIndex == 1) modeSpecificParam = PARAM_OSC_FORMANT_PHASE;
                    break;
                case 3: // Harmonic
                    if (inputIndex == 0) modeSpecificParam = PARAM_OSC_HARMONIC_IDX;
                    else if (inputIndex == 1) modeSpecificParam = PARAM_OSC_HARMONIC_DECAY;
                    else if (inputIndex == 2) modeSpecificParam = PARAM_OSC_HARMONIC_SKEW;
                    break;
            }
            paramId = modeSpecificParam;
        }
        
        // Implement catch-up logic to prevent parameter jumps when switching panels
        if (paramId != PARAM_NONE) {
            float storedKnobValue = knobValues[panelMode][inputIndex];
            float knobPosition = inputs[inputIndex];
            
            // Check if knob has caught up to the stored knob normalizedValue
            if (!knobCaughtUp[inputIndex]) {
                // Check if knob is within threshold of target normalizedValue
                if (fabs(knobPosition - storedKnobValue) < KNOB_CATCHUP_THRESHOLD) {
                    // Knob has caught up - enable tracking
                    knobCaughtUp[inputIndex] = true;
                } else {
                    // Knob hasn't caught up yet - don't update parameter
                    // But still update smoothedKnobState for next comparison
                    previousKnobState[inputIndex] = smoothedKnobState[inputIndex];
                    return; // Early return - don't update anything
                }
            }
            
            // Knob is caught up - update parameter normally
            SetParamValue(paramId, inputs[inputIndex]);
            
            // Update knob normalizedValues storage
            knobValues[panelMode][inputIndex] = inputs[inputIndex];
            
            // Update panel normalizedValues for the currently selected panel (legacy)
            currentPanel.normalizedValues[inputIndex] = inputs[inputIndex];
            displayPanels[panelMode].normalizedValues[inputIndex] = inputs[inputIndex];
            
            knobChanged = true;
        }
    }

    if (currentPanel.id == 'e')
    {
        // ADSR panel - all parameter updates handled by SetParamValue() via bindings
    }
    else if (currentPanel.id == 'm')
    {
        // MIXER panel - all parameter updates handled by SetParamValue() via bindings
    }
    // Removed Pluck panel processing to save memory
    else if (currentPanel.id == 'o')
    {
        // OSC panel - all parameter updates handled by SetParamValue() via bindings
    }
    else if (currentPanel.id == 's')
    {
        // SEQUENCER panel - all parameter updates handled by SetParamValue() via bindings
        // sequencer.sequenceEnabled = sequencer.sequencerMode; // Enable sequence when sequencer mode is active
    }
    else if (currentPanel.id == 't')
    {
        // TUNING panel - all parameter updates handled by SetParamValue() via bindings
    }
    else if (currentPanel.id == 'c')
    {
        // SAMPLER panel - all parameter updates handled by SetParamValue() via bindings
    }

    for (int i = 0; i < 4; i++)
    {
        previousKnobState[i] = smoothedKnobState[i]; // Update with smoothed normalizedValues for next comparison
    }
    
    // Auto-save to SD card when knobs change - DISABLED to prevent audio dropouts
    // Users can manually save via encoder press in PRESET panel
    // if (knobChanged && sdCardInitialized) {
    //     uint32_t currentTime = hw.seed.system.GetNow();
    //     if (currentTime - lastSaveTime >= SAVE_DEBOUNCE_MS) {
    //         RequestSaveToSD();
    //         lastSaveTime = currentTime;
    //     }
    // }
}



void ProcessControls()
{
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    ProcessEncoder();
    ProcessKnobs();
}

void InitPan(float samplerate)
{
    // Initialize manual pan phase tracking
    panPhase = 0.0f;
    panFreq = 0.2f;  // Default panning frequency (0-10Hz range via knob control)
    panAmp = 1.0f;   // Default full amplitude (0-1 range via knob control)
    
    // Keep old pan oscillator initialization for potential future use
    pan.Init(samplerate);
    pan.SetFreq(panFreq);
    pan.SetAmp(1);
    pan.SetWaveform(Oscillator::WAVE_SIN);
}

// seems like there isn't enough processing power to run 4 ouf these in parallel
void plucksApply(float* data) {
    float trig = 0.0f;
    float sig = 0.0f;
    float dry = 1.0f;
    float wet = 0.0f;
    // float trig, nn, decay;       // Pluck Vars
    // float sig, delsig;           // Mono Audio Vars
    // if (envelopes[0].gate)
    // {
    //     trig = 1.0f;
    // } else
    // {
    //     trig = 0.0f;
    // }
    int8_t note = 60;
    // note = voices[0].note;
    // if(hw.encoder.RisingEdge() || hw.gate_input[DaisyPatch::GATE_IN_1].Trig())
    //     trig = 1.0f;
    // // if (envelopes[0].gate)
    // // {
    // //     trig = 1.0f;
    // // }
    // sig = plucks[0].synth.Process(trig, note);

    for(size_t i = 0; i < 1; i++)
    {
        if (envelopes[i].trig)
        {
            trig = 1.0f;
        }
        
        wet = 0.0f; // No pluck wet/dry
        // note = voices[i].note;
        if(hw.encoder.RisingEdge() || hw.gate_input[DaisyPatch::GATE_IN_1].Trig())
            trig = 1.0f;
        // if (envelopes[0].gate)
        // {
        //     trig = 1.0f;
        // }
        if (voices[i].note > 0) {
            // select random note between 48 and 72
            // note = 48 + rand() % 25;
            note = voices[i].note;
        }
        sig = 0.0f; // No pluck signal

        // sig = plucks[i].synth.Process(trig, note);
        // sig = plucks[i].synth.Process(trig, 60);
        dry = 1.0f - wet;
        data[i] = dry * data[i] + wet * sig;
        // data[i] = data[i];
    }
}

// Shift Register Mode Implementation
static void ApplyShiftRegisterState()
{
    const auto& voice_states = shift_register.GetVoices();
    for(size_t i = 0; i < voice_states.size(); ++i)
    {
        const auto& state = voice_states[i];
        if(state.active)
        {
            // Update internal oscillator frequencies BEFORE setting gate to avoid clicks
            if (useInternalOscillators[i]) {
                float freq = MidiNoteToFrequency(static_cast<int8_t>(state.note), 1);  // Use 1 to avoid voice 0 special case
                SetOscillatorFrequency(i, freq);
            }
            
            // Send pitch bend for shift register mode if tuning is enabled
            if (sendPitchBendMidi && state.gate_on) {
                const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                float centsDeviation = CalculateCentsDeviation(static_cast<uint8_t>(state.note), tuning);
                int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                SendPitchBend(static_cast<uint8_t>(i), pitchBendValue);
            }
            
            if(state.needs_retrigger)
            {
                // Set velocity-scaled sustain level before retriggering
                float sustainKnobValue = hw.controls[2].Process(); // Read sustain knob directly
                float baseSustainLevel = 0.01f * powf(100.0f, sustainKnobValue);
                float velocityFactor = state.velocity / 127.0f;
                float velocityScaledSustain = baseSustainLevel * velocityFactor;
                envelopes[i].env.SetSustainLevel(velocityScaledSustain);
                
                envelopes[i].env.Retrigger(true);
            }
            // Use gate_on state from the library (tracks note-on/off)
            envelopes[i].gate  = state.gate_on;
            voices[i].note     = static_cast<int8_t>(state.note);
            voices[i].velocity = static_cast<int8_t>(state.velocity);
        }
        else
        {
            envelopes[i].gate  = false;
            voices[i].note     = 0;
            voices[i].velocity = 0;
        }
    }

    shift_register.ClearRetriggerFlags();
}

void AddNoteToQueue(int8_t note, int8_t velocity)
{
    if(note < 0)
    {
        return;
    }

    const uint8_t midi_note     = static_cast<uint8_t>(note);
    const uint8_t midi_velocity = static_cast<uint8_t>(std::max<int>(0, velocity));
    shift_register.HandleNoteOn(midi_note, midi_velocity);
    ApplyShiftRegisterState();
}

void RemoveNoteFromQueue(int8_t note)
{
    if(note < 0)
    {
        return;
    }

    shift_register.HandleNoteOff(static_cast<uint8_t>(note));
    ApplyShiftRegisterState();
}

void ClearAllVoices()
{
    shift_register.Reset();
    ApplyShiftRegisterState();
}

// Sequencer Mode Implementation
void AddNoteToSequencer(uint8_t note)
{
    // Check if note already exists
    for (size_t i = 0; i < sequencer.sequencerNotes.size(); i++) {
        if (sequencer.sequencerNotes[i] == note) {
            return; // Note already exists, don't add duplicate
        }
    }
    
    // Add note to array
    sequencer.sequencerNotes.push_back(note);
    
    // Sort notes based on current ordering preference
    SortSequencerNotes();
    
    // Reset sequencer note index when new notes are added
    sequencer.sequencerNoteIndex = 0;
    
    // If we're adding notes after initial capture, reset the flag
    // This means sequencer will now stop when notes are released (normal behavior)
    if (sequencer.sequencerUsingInitialCapture) {
        sequencer.sequencerUsingInitialCapture = false;
    }
}

void RemoveNoteFromSequencer(uint8_t note)
{
    // If we're using initially captured notes, preserve all notes in the sequencer
    // This prevents losing notes when they're released at slightly different times
    if (sequencer.sequencerUsingInitialCapture) {
        return; // Don't remove any notes when using initially captured notes
    }
    
    // Find and remove the note (normal behavior for later-added notes)
    for (auto it = sequencer.sequencerNotes.begin(); it != sequencer.sequencerNotes.end(); ++it) {
        if (*it == note) {
            sequencer.sequencerNotes.erase(it);
            break;
        }
    }
    
    // Adjust sequencer note index if needed
    if (!sequencer.sequencerNotes.empty() && sequencer.sequencerNoteIndex >= sequencer.sequencerNotes.size()) {
        sequencer.sequencerNoteIndex = 0;
    }
}

void SortSequencerNotes()
{
    if (sequencer.sequencerNotesAscending) {
        std::sort(sequencer.sequencerNotes.begin(), sequencer.sequencerNotes.end());
    } else {
        std::sort(sequencer.sequencerNotes.begin(), sequencer.sequencerNotes.end(), std::greater<uint8_t>());
    }
}

void ClearSequencerNotes()
{
    sequencer.sequencerNotes.clear();
    sequencer.sequencerNoteIndex = 0;
    sequencer.sequencerUsingInitialCapture = false;  // Reset flag when clearing notes
}

void CaptureCurrentlyHeldNotes()
{
    // Clear existing sequencer notes first
    sequencer.sequencerNotes.clear();
    
    // Capture all currently held notes from the voices array
    for (int i = 0; i < 4; i++) {
        if (envelopes[i].gate && voices[i].note > 0) {
            // Add note to sequencer if it's currently held
            AddNoteToSequencer(static_cast<uint8_t>(voices[i].note));
        }
    }
    
    // Reset sequencer note index to start from the beginning
    sequencer.sequencerNoteIndex = 0;
    
    // Set flag to indicate we're using initially captured notes
    // This means sequencer won't stop when these notes are released
    sequencer.sequencerUsingInitialCapture = true;
}

void ApplyTuningToSequencerNotes()
{
    // Apply tuning changes to all currently playing sequencer voices
    // This ensures that when tuning changes, the sequencer notes reflect the new tuning
    if (!sequencer.sequencerMode || sequencer.sequencerNotes.empty()) {
        return;
    }
    
    // Update pitch bend for all currently active voices that are playing sequencer notes
    for (int i = 0; i < 4; i++) {
        if (envelopes[i].gate && voices[i].note > 0) {
            // Check if this voice is playing a note from the sequencer
            bool isSequencerNote = false;
            for (size_t j = 0; j < sequencer.sequencerNotes.size(); j++) {
                if (voices[i].note == sequencer.sequencerNotes[j]) {
                    isSequencerNote = true;
                    break;
                }
            }
            
            if (isSequencerNote && sendPitchBendMidi) {
                // Apply new tuning to this voice
                const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                float centsDeviation = CalculateCentsDeviation(static_cast<uint8_t>(voices[i].note), tuning);
                int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                SendPitchBend(static_cast<uint8_t>(i), pitchBendValue);
            }
            
            // Update internal oscillator frequency if enabled
            if (useInternalOscillators[i]) {
                float freq = MidiNoteToFrequency(voices[i].note, 1);  // Use 1 to avoid voice 0 special case
                SetOscillatorFrequency(i, freq);
            }
        }
    }
}

// Trigger Sequence Generator Implementation
void InitTriggerSequence()
{
    // Initialize sequence with Euclidean rhythm (4 triggers out of 16 steps)
    GenerateEuclideanRhythm(4, TRIGGER_SEQUENCE_LENGTH, sequencer.triggerSequence);

    // Initialize timing
    sequencer.currentSequenceStep = 0;
    sequencer.lastSequenceStepTime = hw.seed.system.GetNow();
    sequencer.sequenceStepInterval = static_cast<uint32_t>(60000 / (sequencer.clockBpm * 4));  // 16th note timing
    
    // Initialize subdivision timing (for CC multiplier feature)
    sequencer.currentSubdivision = 0;
    sequencer.lastSubdivisionTime = hw.seed.system.GetNow();
    sequencer.subdivisionInterval = sequencer.sequenceStepInterval / 8;
}


// Legacy wrapper functions for backward compatibility
void AddCCToQueue(uint8_t ccValue, bool isReset) {
    // Check if queue is full
    if (ccQueueCount >= CC_QUEUE_SIZE) {
        // SetDebugMessage("CC Queue Full!");
        return;
    }
    
    uint32_t currentTime = hw.seed.system.GetNow();
    uint32_t sendTime;
    
    if (!isReset) {
        if (ccQueueCount == 0) {
            sendTime = currentTime + 10;
        } else {
            sendTime = currentTime + (ccQueueCount * CC_RESET_DELAY_MS) + 10;
        }
    } else {
        sendTime = currentTime + CC_RESET_DELAY_MS;
    }
    
    uint32_t holdUntil = sendTime + CC_RESET_DELAY_MS;
    
    ccQueue[ccQueueTail].ccValue = ccValue;
    ccQueue[ccQueueTail].sendTime = sendTime;
    ccQueue[ccQueueTail].holdUntil = holdUntil;
    ccQueue[ccQueueTail].isReset = isReset;
    
    ccQueueTail = (ccQueueTail + 1) % CC_QUEUE_SIZE;
    ccQueueCount++;
}

void ProcessCCQueue() {
    uint32_t currentTime = hw.seed.system.GetNow();
    
    // Initialize CC state if not done yet
    if (!ccStateInitialized) {
        SendMidiMesssage(0, 15, "CC");
        lastCCValue = 0;
        ccIsLatched = false;
        ccStateInitialized = true;
        // SetDebugMessage("CC Init");
    }
    
    // Process only the next ready item in the queue (one at a time to respect timing)
    if (ccQueueCount > 0) {
        CCQueueItem& item = ccQueue[ccQueueHead];
        
        if (currentTime >= item.sendTime) {
            // Check if enough time has passed since last trigger-on to prevent double-triggering
            uint32_t timeSinceLastTrigger = currentTime - ccLatchTime;
            if (timeSinceLastTrigger >= CC_RESET_DELAY_MS || !ccIsLatched) {
                SendMidiMesssage(item.ccValue, 15, "CC");
                lastCCValue = item.ccValue;
                ccLatchTime = currentTime;
                ccIsLatched = true;

                SendMidiMesssage(sequencer.triggerNote, sequencer.ccTriggerChannel, "TRIGGER_OFF");
                SendMidiMesssage(sequencer.triggerNote, sequencer.ccTriggerChannel, "TRIGGER_ON");
                // ccTriggerOffTime = currentTime + TRIGGER_OFF_DELAY_MS;
                ccTriggerOffTime = currentTime + CC_RESET_DELAY_MS;
                ccTriggerOffPending = true;
                
                ccQueueHead = (ccQueueHead + 1) % CC_QUEUE_SIZE;
                ccQueueCount--;
            }
        }
    }
    
    // Check if CC should be latched down to zero (using same delay as CC changes)
    if (ccIsLatched && ccQueueCount == 0) {
        uint32_t timeSinceLastCC = currentTime - ccLatchTime;
        if (timeSinceLastCC >= CC_RESET_DELAY_MS) {
            SendMidiMesssage(0, sequencer.ccValueChannel, "CC");
            lastCCValue = 0;
            ccIsLatched = false;
            // SetDebugMessage("CC Latch Down");
        }
    }
}

void ResetCCState() {
    // Force immediate reset to clean state
    ccQueueCount = 0;
    ccQueueHead = 0;
    ccQueueTail = 0;
    SendMidiMesssage(0, sequencer.ccValueChannel, "CC");
    lastCCValue = 0;
    ccIsLatched = false;
    ccStateInitialized = true;
    // SetDebugMessage("CC Reset");
}

bool ShouldFireWithProbability(uint8_t probability) {
    if (probability >= 100) return true;
    if (probability == 0) return false;
    uint8_t randomValue = rand() % 100; // 0-99
    return randomValue < probability;
}

void ProcessCCSlots()
{
    globalNoteCounter++; // Keep for display purposes
    
    int triggeredSlots = 0;
    uint8_t triggeredValues[8];
    
    // Check each slot's probability (reverse order to prioritize CC8)
    for (int i = 7; i >= 0; i--) {
        if (ShouldFireWithProbability(ccSlotProbabilities[i])) {
            triggeredValues[triggeredSlots] = ccSlotValues[i];
            triggeredSlots++;
        }
    }
    
    // Queue all triggered CCs
    for (int i = 0; i < triggeredSlots; i++) {
        AddCCToQueue(triggeredValues[i], false);
        // AddCCToQueue(ccSlotValues[7], true);
    }
    
    // If no CCs were triggered and we have a high CC normalizedValue, force latch down
    if (triggeredSlots == 0 && lastCCValue > 0) {
        ResetCCState();
    }
}

// Process CC slots with subdivision support for multipliers
// subdivision should be 0-7 (one of 8 subdivisions of a sequencer step)
void ProcessCCSlotsWithSubdivision(uint8_t subdivision)
{
    globalNoteCounter++; // Keep for display purposes
    
    int triggeredSlots = 0;
    uint8_t triggeredValues[8];
    
    // Check each slot to see if it should trigger on this subdivision
    // based on its multiplier
    for (int i = 7; i >= 0; i--) {
        uint8_t multiplier = ccSlotMultipliers[i];
        
        // Determine if this slot should trigger on this subdivision
        // multiplier 1: subdivision 0 only (subdivision % 8 == 0)
        // multiplier 2: subdivisions 0, 4 (subdivision % 4 == 0)
        // multiplier 4: subdivisions 0, 2, 4, 6 (subdivision % 2 == 0)
        // multiplier 8: all subdivisions 0-7 (always trigger)
        bool shouldTrigger = false;
        if (multiplier == 1) {
            shouldTrigger = (subdivision == 0); // Only trigger once at start of step
        } else {
            // Formula: (subdivision % (8 / multiplier)) == 0
            // This gives evenly spaced triggers based on multiplier
            uint8_t divisor = 8 / multiplier;
            shouldTrigger = (subdivision % divisor) == 0;
        }
        
        // If this slot should trigger on this subdivision, evaluate its probability
        if (shouldTrigger && ShouldFireWithProbability(ccSlotProbabilities[i])) {
            triggeredValues[triggeredSlots] = ccSlotValues[i];
            triggeredSlots++;
        }
    }
    
    // Queue all triggered CCs
    for (int i = 0; i < triggeredSlots; i++) {
        AddCCToQueue(triggeredValues[i], false);
    }
    
    // If no CCs were triggered and we have a high CC normalizedValue, force latch down
    if (triggeredSlots == 0 && lastCCValue > 0) {
        // Only reset at the end of a complete step (subdivision 7)
        if (subdivision == 7) {
            ResetCCState();
        }
    }
}

void GenerateEuclideanRhythm(int numTriggers, int numSteps, bool* pattern)
{
    // Handle edge cases
    if (numTriggers <= 0) {
        // No triggers - fill with false
        for (int i = 0; i < numSteps; i++) {
            pattern[i] = false;
        }
        return;
    }

    if (numTriggers >= numSteps) {
        // All steps are triggers - fill with true
        for (int i = 0; i < numSteps; i++) {
            pattern[i] = true;
        }
        return;
    }

    // Bjorklund algorithm implementation
    std::vector<int> remainder;
    std::vector<int> count;
    
    remainder.push_back(numTriggers);
    int divisor = numSteps - numTriggers;
    int level = 0;
    
    // Build the remainder and count arrays
    do {
        count.push_back(divisor / remainder[level]);
        remainder.push_back(divisor % remainder[level]);
        divisor = remainder[level];
        level++;
    } while (remainder[level] > 1);

    count.push_back(divisor);

    // Build the pattern using the Bjorklund algorithm
    std::vector<bool> result;
    BuildPattern(level, result, count, remainder);

    // Copy result to pattern array
    for (int i = 0; i < numSteps; i++) {
        pattern[i] = result[i];
    }
}

void BuildPattern(int level, std::vector<bool>& result, const std::vector<int>& count, const std::vector<int>& remainder)
{
    if (level == -1) {
        result.insert(result.begin(), false);
    } else if (level == -2) {
        result.insert(result.begin(), true);
    } else {
        for (int i = 0; i < count[level]; i++) {
            BuildPattern(level - 1, result, count, remainder);
        }
        if (remainder[level] != 0) {
            BuildPattern(level - 2, result, count, remainder);
        }
    }
}

// ============================================================================
// MIDI Architecture - Template-based Handler Chain Implementation
// ============================================================================

// NullHandler terminates the chain
struct NullHandler {
    bool HandleNoteOn(NoteOnEvent& event) { return false; }
    bool HandleNoteOff(NoteOffEvent& event) { return false; }
};

// Template-based handler chain - zero runtime overhead
template<typename NextHandler>
class HandlerBase {
protected:
    NextHandler next;
public:
    NextHandler& GetNext() { return next; }
};

// SequencerCaptureHandler - captures notes into sequencer array
template<typename NextHandler>
class SequencerCaptureHandler : public HandlerBase<NextHandler> {
public:
    bool HandleNoteOn(NoteOnEvent& event) {
        if (sequencer.sequencerMode) {
            // Add note to sequencer notes array
            AddNoteToSequencer(event.note);
        }
        // Always pass to next handler
        return this->GetNext().HandleNoteOn(event);
    }
    
    bool HandleNoteOff(NoteOffEvent& event) {
        if (sequencer.sequencerMode) {
            // Remove note from sequencer notes array
            RemoveNoteFromSequencer(event.note);
        }
        // Always pass to next handler
        return this->GetNext().HandleNoteOff(event);
    }
};

// ShiftRegisterHandler - processes notes through shift register system
template<typename NextHandler>
class ShiftRegisterHandler : public HandlerBase<NextHandler> {
public:
    bool HandleNoteOn(NoteOnEvent& event) {
        if (shiftRegisterMode) {
            // Process through shift register
            AddNoteToQueue(event.note, event.velocity);
            
            // Send MIDI to external devices with tuning applied
            if (sendPitchBendMidi) {
                const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                float centsDeviation = CalculateCentsDeviation(event.note, tuning);
                int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                SendPitchBend(event.channel, pitchBendValue);
            }
            
            uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + event.channel), event.note, event.velocity};
            hw.midi.SendMessage(bytes, 3);
            
            return true; // Handled - stop chain
        }
        // Pass to next handler if not in shift register mode
        return this->GetNext().HandleNoteOn(event);
    }
    
    bool HandleNoteOff(NoteOffEvent& event) {
        if (shiftRegisterMode) {
            // Process through shift register
            RemoveNoteFromQueue(event.note);
            
            // Send note-off to external devices
            uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + event.channel), event.note, event.velocity};
            hw.midi.SendMessage(bytes, 3);
            
            return true; // Handled - stop chain
        }
        // Pass to next handler if not in shift register mode
        return this->GetNext().HandleNoteOff(event);
    }
};

// NormalVoiceHandler - direct voice allocation (round-robin)
template<typename NextHandler>
class NormalVoiceHandler : public HandlerBase<NextHandler> {
public:
    bool HandleNoteOn(NoteOnEvent& event) {
        // Voice allocation: Round-robin distribution across voices 0-3
        int8_t voiceIndex = -1;
        
        // Step 1: Try the next voice in round-robin sequence if it's free
        if (!envelopes[nextVoiceIndex].gate) {
            voiceIndex = nextVoiceIndex;
        } else {
            // Step 2: If next voice is busy, search for any free voice
            bool foundFree = false;
            for (int i = 0; i < 4; i++) {
                if (!envelopes[i].gate) {
                    voiceIndex = i;
                    foundFree = true;
                    break;
                }
            }
            
            // Step 3: If no free voice, use the next voice in round-robin (voice stealing)
            if (!foundFree) {
                voiceIndex = nextVoiceIndex;
                
                // Send note-off for the stolen voice
                if (voices[voiceIndex].note > 0) {
                    uint8_t noteOffBytes[3] = {
                        static_cast<uint8_t>(0x80 + voiceIndex), 
                        static_cast<uint8_t>(voices[voiceIndex].note), 
                        0
                    };
                    hw.midi.SendMessage(noteOffBytes, 3);
                }
            }
        }
        
        // Store the allocated voice channel
        event.channel = voiceIndex;
        
        // Advance round-robin index for next note
        nextVoiceIndex = (voiceIndex + 1) % 4;
        
        // Send pitch bend before note-on if tuning is enabled
        if (sendPitchBendMidi) {
            const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
            float centsDeviation = CalculateCentsDeviation(event.note, tuning);
            int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
            SendPitchBend(voiceIndex, pitchBendValue);
        }
        
        // Send MIDI note-on to external devices on the allocated voice channel
        uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + voiceIndex), event.note, event.velocity};
        hw.midi.SendMessage(bytes, 3);
        
        // Update internal oscillator frequencies BEFORE setting gate to avoid clicks
        if (useInternalOscillators[voiceIndex]) {
            float freq = MidiNoteToFrequency(event.note, 1);  // Use 1 to avoid voice 0 special case
            SetOscillatorFrequency(voiceIndex, freq);
        }
        
        // Update voice state
        envelopes[voiceIndex].gate = true;
        voices[voiceIndex].note = event.note;
        voices[voiceIndex].velocity = event.velocity;
        voices[voiceIndex].allocationOrder = voiceAllocationCounter++;
        
        // Set velocity-scaled sustain level before retriggering
        float sustainKnobValue = hw.controls[2].Process(); // Read sustain knob directly
        float baseSustainLevel = 0.01f * powf(100.0f, sustainKnobValue);
        float velocityFactor = event.velocity / 127.0f;
        float velocityScaledSustain = baseSustainLevel * velocityFactor;
        envelopes[voiceIndex].env.SetSustainLevel(velocityScaledSustain);
        
        envelopes[voiceIndex].env.Retrigger(true);
        
        // Pass to next handler
        return this->GetNext().HandleNoteOn(event);
    }
    
    bool HandleNoteOff(NoteOffEvent& event) {
        // Voice allocation: turn off all voices playing this note
        for (int i = 0; i < 4; i++) {
            if (voices[i].note == event.note) {
                // Send MIDI note-off to external devices on this voice's channel
                uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + i), event.note, event.velocity};
                hw.midi.SendMessage(bytes, 3);
                
                envelopes[i].gate = false;
                
                // Clear voice data when note is released
                voices[i].note = 0;
                voices[i].velocity = 0;
                voices[i].allocationOrder = 0;
            }
        }
        
        // Pass to next handler
        return this->GetNext().HandleNoteOff(event);
    }
};

// IntellijelPitchHandler - handles current note processing for channel 15
// processes the incoming note and sends it to Multigrain via channel 15
template<typename NextHandler>
class IntellijelPitchHandler : public HandlerBase<NextHandler> {
public:
    bool HandleNoteOn(NoteOnEvent& event) {
        // Turn off the previously played note first
        if (lastCurrentNote != 0) {
            SendMidiMesssage(lastCurrentNote, 15, "NOTE_OFF");
        }

        // this voice is meant to be sent to Multigrain
        // pass current note and trigger to Intellijel via channel 15
        lastCurrentNote = currentNote;
        currentNote = event.note;
        
        // Send the new note
        SendMidiMesssage(event.note, 15, "NOTE_ON");
        
        // Process CC slots based on subdivision logic
        if (sequencer.sequencerMode) {
            // ProcessCCSlotsWithSubdivision(sequencer.currentSubdivision);
        } else {
            ProcessCCSlots();
        }
        
        noteCount++;
        
        // Always pass to next handler (side effects, doesn't stop chain)
        return this->GetNext().HandleNoteOn(event);
    }
    
    bool HandleNoteOff(NoteOffEvent& event) {
        // Always pass to next handler (side effects, doesn't stop chain)
        return this->GetNext().HandleNoteOff(event);
    }
};

// IntellijelTrackerHandler - tracks highest/lowest notes and sends to Intellijel
// needs to be sent AFTER voice allocation so it can access complete voice state
template<typename NextHandler>
class IntellijelTrackerHandler : public HandlerBase<NextHandler> {
public:
    bool HandleNoteOn(NoteOnEvent& event) {
        // Pass highest currently held note to Intellijel via channel 14
        currentHighestNote = getCurrentHighestNote();
        if (currentHighestNote != lastHighestNote) {
            SendMidiMesssage(currentHighestNote, 14, "NOTE_ON");

            // turn off previous note
            if (lastHighestNote != 0 && lastHighestNote != currentHighestNote) {
                SendMidiMesssage(lastHighestNote, 14, "NOTE_OFF");
            }

            lastHighestNote = currentHighestNote;
        }
        
        // pass lowest currently held note to Intellijel via channel 13
        currentLowestNote = getCurrentLowestNote();
        if (currentLowestNote != lastLowestNote) {
            SendMidiMesssage(currentLowestNote, 13, "NOTE_ON");

            // turn off previous note
            if (lastLowestNote != 0 && lastLowestNote != currentLowestNote) {
                SendMidiMesssage(lastLowestNote, 13, "NOTE_OFF");
            }
        }
        
        lastLowestNote = currentLowestNote;
        
        // Always pass to next handler (side effects, doesn't stop chain)
        return this->GetNext().HandleNoteOn(event);
    }
    
    bool HandleNoteOff(NoteOffEvent& event) {
        return this->GetNext().HandleNoteOff(event);
    }
};

// Define the handler chain type - compile-time composition
using HandlerChain = SequencerCaptureHandler<
    ShiftRegisterHandler<
        IntellijelPitchHandler<
            NormalVoiceHandler<
                IntellijelTrackerHandler<NullHandler>
            >
        >
    >
>;

// Global handler chain instance
HandlerChain handlerChain;

// Wrapper functions for forward declarations
void ProcessHandlerChainNoteOn(NoteOnEvent& event) {
    handlerChain.HandleNoteOn(event);
}

void ProcessHandlerChainNoteOff(NoteOffEvent& event) {
    handlerChain.HandleNoteOff(event);
}

// SequencerMidiSource - generates notes from sequencer array based on clock
class SequencerMidiSource {
public:
    void Process() {
        if (!sequencer.sequencerMode || sequencer.sequencerNotes.empty()) {
            return;
        }
        
        // Check if it's time for the next subdivision
        uint32_t currentTime = hw.seed.system.GetNow();
        
        // Check subdivision timing (every 1/8 of a sequencer step)
        if ((currentTime - sequencer.lastSubdivisionTime) >= sequencer.subdivisionInterval) {
            // Only process CC slots if current step should trigger a note
            bool currentStepTriggers = sequencer.triggerSequence[sequencer.currentSequenceStep];
            
            if (currentStepTriggers) {
                // Process CC slots for this subdivision
                ProcessCCSlotsWithSubdivision(sequencer.currentSubdivision);
            }
            
            sequencer.lastSubdivisionTime = currentTime;
            
            // Advance to next subdivision
            sequencer.currentSubdivision++;
            
            // When we've completed all 8 subdivisions, advance to next sequence step
            if (sequencer.currentSubdivision >= 8) {
                sequencer.currentSubdivision = 0;
                AdvanceSequenceStep();  // Only triggers note if step has trigger
                sequencer.lastSequenceStepTime = currentTime;
                
                // Start fresh at subdivision 0 for the new step
                sequencer.lastSubdivisionTime = currentTime;  // Reset subdivision timer
            }
        }
    }
    
private:
    void AdvanceSequenceStep() {
        // Check if current step should trigger
        if (sequencer.triggerSequence[sequencer.currentSequenceStep]) {
            // Sequencer mode: trigger next note from sequencer notes array
            uint8_t noteToTrigger = sequencer.sequencerNotes[sequencer.sequencerNoteIndex];
            
            // Create note event and send through handler chain
            NoteOnEvent event;
            event.note = noteToTrigger;
            event.velocity = 127;
            event.channel = 0; // Will be set by voice allocation
            
            ProcessHandlerChainNoteOn(event);
            
            // Note: CC processing happens via subdivision loop, not here
            // This ensures proper timing aligned with subdivisions
            
            // Advance to next note in sequencer array
            sequencer.sequencerNoteIndex = (sequencer.sequencerNoteIndex + 1) % sequencer.sequencerNotes.size();
            
            // Schedule note-off based on note length percentage (only when not using shift register)
            if (!shiftRegisterMode) {
                uint32_t noteOffDelay = static_cast<uint32_t>(sequencer.sequenceStepInterval * sequencer.sequencerNoteLengthPercent);
                noteOffDelay = std::min(noteOffDelay, sequencer.sequenceStepInterval - 10); // Leave at least 10ms before next step
                sequencer.sequencerNoteOffTime = hw.seed.system.GetNow() + noteOffDelay;
                sequencer.sequencerNoteOffPending = true;
                sequencer.sequencerNoteToTurnOff = noteToTrigger;
                sequencer.sequencerVoiceToTurnOff = event.channel; // Will be set by voice allocation
            }
        } else {
            // Original trigger behavior for keyboard mode or when no sequencer notes
            // Use CC queue system instead of direct SendMidiMesssage to ensure proper state management
            // AddCCToQueue(127, false);
            // SendMidiMesssage(triggerNote, 15, "TRIGGER_ON");

            // Schedule trigger off after a short duration (50ms)
            // sequenceTriggerOffTime = hw.seed.system.GetNow() + 50;
            // sequenceTriggerOffPending = true;
        }
        
        // Advance to next step
        sequencer.currentSequenceStep = (sequencer.currentSequenceStep + 1) % TRIGGER_SEQUENCE_LENGTH;
    }
};

// Global sequencer source instance
SequencerMidiSource sequencerMidiSource;

// Wrapper function for sequencer source
void ProcessSequencerMidiSource() {
    sequencerMidiSource.Process();
}

// Common preset storage constants
static const int PRESET_SECTOR = 2000;
static const int PRESET_BUFFER_LENGTH = 128;
static const int PRESET_TIMEOUT_MS = 5000;
static const int PRESET_MAX_RETRIES = 3;

bool SavePreset() {
    uint32_t buffer[PRESET_BUFFER_LENGTH];
    memset(buffer, 0, sizeof(buffer));
    
    // Store all 32 knob normalizedValues (8 panels × 4 knobs)
    int idx = 0;
    for (int panel = 0; panel < panelModesCount; panel++) {
        for (int knob = 0; knob < 4; knob++) {
            int intValue = (int)(knobValues[panel][knob] * 100);
            buffer[idx++] = (uint32_t)intValue;
        }
    }
    
    // Retry on write failure (retry count = 1)
    uint8_t result;
    for (int retry = 0; retry < 2; retry++) { // 2 attempts = 1 retry
        result = BSP_SD_WriteBlocks(buffer, PRESET_SECTOR, 1, 2000); // 2 second timeout
        if (result == MSD_OK) {
            SetDebugMessage("saved");
            return true;
        }
    }
    
    SetDebugMessageF("save fail: %d", result);
    return false;
}

bool LoadPreset() {
    uint32_t buffer[PRESET_BUFFER_LENGTH];
    uint8_t result = BSP_SD_ReadBlocks(buffer, PRESET_SECTOR, 1, PRESET_TIMEOUT_MS);
    if (result != MSD_OK) {
        SetDebugMessageF("load fail: %d", result);
        return false;
    }
    
    // Load all knob normalizedValues (8 panels × 4 knobs)
    int idx = 0;
    for (int panel = 0; panel < panelModesCount; panel++) {
        for (int knob = 0; knob < 4; knob++) {
            int intValue = (int)buffer[idx++];
            knobValues[panel][knob] = intValue / 100.0f;
        }
    }
    
    SetDebugMessage("loaded");
    return true;
}