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
#include "Microsound.h"

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
Limiter         limiter;
microsound::PulsarSynth pulsarSynth;
int8_t microsoundNote = 60;  // Current MIDI note for microsound

// Delay lines for stereo delay effect
// Maximum delay: 600ms at 48kHz = 28800 samples, use 30000 for safety margin
static constexpr size_t MAX_DELAY_SAMPLES = 30000;
DelayLine<float, MAX_DELAY_SAMPLES> delayL;
DelayLine<float, MAX_DELAY_SAMPLES> delayR;
float delaySampleRate = 48000.0f;
float currentDelayTimeSamples = 0.0f;  // Cached delay time in samples

// Microsound ring buffer - separate from delay for stability
// 500ms buffer at 48kHz = 24000 samples (enough for 50-200ms grain reads)
static constexpr size_t MICROSOUND_BUFFER_SIZE = 24000;
DelayLine<float, MICROSOUND_BUFFER_SIZE> microsoundBufferL;
DelayLine<float, MICROSOUND_BUFFER_SIZE> microsoundBufferR;

// Resampling delay read positions (offset from write pointer in samples, can be fractional)
float delayReadPosL = 0.0f;  // Current read position offset for left channel
float delayReadPosR = 0.0f;  // Current read position offset for right channel

// One-pole filter states for pluck-like damping (separate for stereo)
float delayFilterStateL = 0.0f;
float delayFilterStateR = 0.0f;

// Audio-rate timing utilities
static volatile uint32_t gAudioSampleCounterLow = 0;
static volatile uint32_t gAudioSampleCounterHigh = 0;
static float gAudioSampleRate = 48000.0f;

inline float GetAudioSampleRate()
{
    return (gAudioSampleRate > 0.0f) ? gAudioSampleRate : 48000.0f;
}

inline void SetAudioSampleRate(float sampleRate)
{
    gAudioSampleRate = sampleRate;
}

inline void ResetAudioSampleCounter()
{
    gAudioSampleCounterLow = 0;
    gAudioSampleCounterHigh = 0;
}

inline void IncrementAudioSampleCounter(uint32_t delta)
{
    uint32_t newLow = gAudioSampleCounterLow + delta;
    if (newLow < gAudioSampleCounterLow)
    {
        gAudioSampleCounterHigh++;
    }
    gAudioSampleCounterLow = newLow;
}

inline uint64_t ReadAudioSampleCounter()
{
    uint32_t high1, low, high2;
    do
    {
        high1 = gAudioSampleCounterHigh;
        low = gAudioSampleCounterLow;
        high2 = gAudioSampleCounterHigh;
    } while (high1 != high2);
    return (static_cast<uint64_t>(high1) << 32) | low;
}

// ============================================================================
// Parameter Configuration - Compile-Time Constants (Zero RAM Usage)
// ============================================================================

namespace ParamConfig {
    // Delay parameters
    static constexpr float DELAY_SHORT_MIN_MS = 0.05f;
    static constexpr float DELAY_SHORT_MAX_MS = 25.0f;
    static constexpr float DELAY_SHORT_RANGE_MS = DELAY_SHORT_MAX_MS - DELAY_SHORT_MIN_MS;
    
    static constexpr float BEAT_MULTIPLIERS[7] = {0.25f, 0.333333f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f};
    static constexpr int NUM_BEAT_MULTIPLIERS = 7;
    
    // ADSR parameters
    static constexpr float ADSR_MIN_MS = 0.1f;
    static constexpr float ADSR_MAX_MS = 1500.0f;
    
    // Pan parameters
    static constexpr float PAN_FREQ_MAX_HZ = 10.0f;
    
    // Sequencer parameters
    static constexpr int SEQ_DENSITY_MAX = 16;
    static constexpr int SEQ_ORDER_MODES = 6;
}

// ============================================================================
// String Constants - Shared to Reduce Flash Usage
// ============================================================================
static const char* STR_ASC = "ASC";
static const char* STR_DESC = "DESC";
static const char* STR_UPD = "UPD";
static const char* STR_FWD = "FWD";
static const char* STR_RND = "RND";
static const char* STR_BRN = "BRN";
static const char* STR_ON = "ON";
static const char* STR_OFF = "OFF";
static const char* STR_SINE = "Sine";
static const char* STR_TRI = "Tri";
static const char* STR_SQR = "Sqr";
static const char* STR_SAW = "Saw";
static const char* STR_INTER = "Inter";
static const char* STR_SHORT = "SHORT";
static const char* STR_LONG = "LONG";
static const char* STR_DENS = "Dens";
static const char* STR_ORD = "Ord";
static const char* STR_CCP = "CC%";
static const char* STR_OFFSET = "Off";
static const char* STR_MULT = "Mult";

// ============================================================================
// Parameter Transformation Helpers - Inline (Zero Function Call Overhead)
// ============================================================================

// Linear mapping: normalized (0-1) -> value (min-max)
inline constexpr float LinearMap(float normalized, float min, float max) {
    return min + (normalized * (max - min));
}

// Exponential mapping: normalized (0-1) -> value (min-max) with exponential curve
inline constexpr float ExpMap(float normalized, float min, float max) {
    return min + ((normalized * normalized) * (max - min));
}

// Logarithmic mapping: normalized (0-1) -> value (min-max) with log curve
inline float LogMap(float normalized, float min, float max) {
    return min * powf(max / min, normalized);
}

// Percentage formatter (0-100%) - uses static buffer to avoid allocations
inline const char* FormatPercent(float normalized) {
    static char buf[8];
    snprintf(buf, sizeof(buf), "%d%%", static_cast<int>(normalized * 100));
    return buf;
}

// Milliseconds formatter - uses static buffer to avoid allocations
inline const char* FormatMs(float ms) {
    static char buf[12];
    if (ms < 1.0f) {
        int msInt = static_cast<int>(ms * 100);
        if (msInt < 10) {
            snprintf(buf, sizeof(buf), "0.0%dms", msInt);
        } else {
            snprintf(buf, sizeof(buf), "0.%dms", msInt);
        }
    } else {
        snprintf(buf, sizeof(buf), "%dms", static_cast<int>(ms));
    }
    return buf;
}

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
    
    void SetPhase(float phase) {
        phase_ = phase;
        // Keep phase in 0-1 range
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        if (phase_ < 0.0f) phase_ += 1.0f;
    }
    
    float GetPhase() const {
        return phase_;
    }
    
    // Generate output at a specific phase without advancing internal phase
    float ProcessAtPhase(float phase) const {
        // Normalize phase to 0-1 range
        while (phase >= 1.0f) phase -= 1.0f;
        while (phase < 0.0f) phase += 1.0f;
        
        float output = 0.0f;
        
        // Generate base waveforms
        float sine = sinf(phase * 2.0f * M_PI);
        float triangle = 2.0f * (phase < 0.5f ? 2.0f * phase : 2.0f * (1.0f - phase)) - 1.0f;
        float square = phase < 0.5f ? 1.0f : -1.0f;
        float saw = 2.0f * phase - 1.0f;
        
        // Interpolate between waveforms
        // Clamp waveform_param_ to valid range to prevent crashes
        float wp = std::max(0.0f, std::min(1.0f, waveform_param_));

        // wp = 0.0f;
        
        // Explicit handling for pure waveforms to avoid floating point issues
        if (wp < 0.001f) {
            // Pure sine (wp == 0.0 or very close)
            output = sine;
        } else if (wp <= 0.33f) {
            // Interpolate between sine and triangle
            float t = wp / 0.33f;
            output = sine * (1.0f - t) + triangle * t;
        } else if (wp <= 0.66f) {
            // Interpolate between triangle and square
            float t = (wp - 0.33f) / 0.33f;
            output = triangle * (1.0f - t) + square * t;
        } else if (wp > 0.999f) {
            // Pure sawtooth (wp == 1.0 or very close)
            output = saw;
        } else {
            // Interpolate between square and saw
            float t = (wp - 0.66f) / 0.34f;
            output = square * (1.0f - t) + saw * t;
        }
        
        return output * amp_;
    }
    
    float Process() {
        float output = 0.0f;
        
        // Generate base waveforms
        float sine = sinf(phase_ * 2.0f * M_PI);
        float triangle = 2.0f * (phase_ < 0.5f ? 2.0f * phase_ : 2.0f * (1.0f - phase_)) - 1.0f;
        float square = phase_ < 0.5f ? 1.0f : -1.0f;
        float saw = 2.0f * phase_ - 1.0f;
        
        // Interpolate between waveforms
        // Clamp waveform_param_ to valid range to prevent crashes
        float wp = std::max(0.0f, std::min(1.0f, waveform_param_));
        
        // Explicit handling for pure waveforms to avoid floating point issues
        if (wp < 0.001f) {
            // Pure sine (wp == 0.0 or very close)
            output = sine;
        } else if (wp <= 0.33f) {
            // Interpolate between sine and triangle
            float t = wp / 0.33f;
            output = sine * (1.0f - t) + triangle * t;
        } else if (wp <= 0.66f) {
            // Interpolate between triangle and square
            float t = (wp - 0.33f) / 0.33f;
            output = triangle * (1.0f - t) + square * t;
        } else if (wp > 0.999f) {
            // Pure sawtooth (wp == 1.0 or very close)
            output = saw;
        } else {
            // Interpolate between square and saw
            float t = (wp - 0.66f) / 0.34f;
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
HarmonicOscillator<> voiceHarmonicOsc[4];    // Harmonic oscillators for all 4 voices (default 16 harmonics)
InterpolatedOscillator panLfo;              // LFO for panning CV output
InterpolatedOscillator internalPhaseOsc[4]; // Internal oscillators for phase generation (voices 1 and 3 use these)
// Binaural state variables - using simple phase accumulators instead of heavy InterpolatedOscillator
bool binauralEnabled = false;
float binauralSpreadHz = 0.0f;
float binauralPhase[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float binauralFreqHz[4] = {440.0f, 440.0f, 440.0f, 440.0f};

// Sine lookup table for fast binaural oscillator (256 entries, no trig calls needed)
static float sineLUT[256];
static bool sineLUTInitialized = false;

static inline float fastSin(float phase) {
    // phase is 0.0 to 1.0, returns sine value from lookup table
    int idx = static_cast<int>(phase * 255.99f) & 255;
    return sineLUT[idx];
}

// Equal power panning lookup tables (256 entries each, no trig calls needed)
static float panLeftLUT[256];   // cos coefficients
static float panRightLUT[256];  // sin coefficients

static inline void fastPanEqualPower(float pan, float value, float* left, float* right) {
    // pan is -1.0 to +1.0, map to 0-255
    int idx = static_cast<int>((pan + 1.0f) * 127.5f);
    if (idx < 0) idx = 0;
    if (idx > 255) idx = 255;
    *left = value * panLeftLUT[idx];
    *right = value * panRightLUT[idx];
}

size_t blocksize = 16; //(333us latency vs 167us for 8)
int panelMode;
float voicesMinLevel = 0.0f;

// float panOutput;
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

bool shiftRegisterMode = false;

// option to use internal oscillators for each voice
bool useInternalOscillators[4] = {false, true, false, true};

// Phase tracking for external audio inputs (voices 0 and 2) to handle discontinuities
float externalPhaseTrack[2] = {0.0f, 0.0f};  // Tracked phase for voices 0 and 2
float externalPhasePrev[2] = {0.0f, 0.0f};   // Previous mapped phase for discontinuity detection
float externalPhaseSmooth[2] = {0.0f, 0.0f};  // Smoothed phase to reduce clicks

// Peak amplitude tracking for input normalization (voices 0 and 2)
// This ensures we always use the full 0-1 phase range regardless of input amplitude
float inputPeakAmplitude[2] = {1.0f, 1.0f};  // Tracked peak amplitude for normalization
float normalizedInputPrev[2] = {0.0f, 0.0f};  // Previous normalized input for smoothing

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
template<typename NextHandler> class ControlHandler;

void ProcessHandlerChainNoteOn(NoteOnEvent& event);
void ProcessHandlerChainNoteOff(NoteOffEvent& event);
void ProcessHandlerChainControlChange(ControlChangeEvent& event);
void ProcessSequencerMidiSource();
void AdvanceSequencerStep(); // Advance sequencer step (kept for compatibility)
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
const uint32_t CC_RESET_DELAY_MS = 10; // drops signal sometimes below 15ms
uint8_t lastCCValue = 0; // Track the last CC normalizedValue sent (start with lowest CC normalizedValue)
bool ccStateInitialized = false; // Track if CC state has been properly initialized

// CC Slot System variables
uint8_t ccSlotValues[8] = {0, 18, 36, 54, 73, 91, 109, 127}; // Equally distributed CC normalizedValues 0-127
uint8_t ccSlotProbabilities[8] = {100, 100, 100, 100, 100, 100, 100, 100}; // Default all probabilities to 100%
uint32_t globalNoteCounter = 0; // Increments on every note-on

// CC Queue System
struct CCQueueItem {
    uint8_t ccValue;
    bool isReset; // true if this is a reset to lowest normalizedValue
};

const size_t CC_QUEUE_SIZE = 8; // Reduced to save memory (was 16)
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

// Sequencer order modes
enum SequencerOrderMode {
    SEQ_ORDER_ASC = 0,    // Ascending
    SEQ_ORDER_DESC = 1,   // Descending
    SEQ_ORDER_UPD = 2,    // Up-down
    SEQ_ORDER_FWD = 3,    // Forward (original order)
    SEQ_ORDER_RND = 4,    // Random
    SEQ_ORDER_BRN = 5     // Brownian
};

// Individual sequencer track state
struct SequencerTrack {
    // Track-specific parameters
    float density = 0.0f;  // 0.0f to 16.0f - determines number of notes in Euclidean rhythm pattern
    SequencerOrderMode orderMode = SEQ_ORDER_ASC;
    float noteLengthPercent = 0.5f;  // Note length as percentage of step (10%-90%)
    float offset = 0.0f;  // Offset in semitones (-12, -5, 0, 5, 12)
    
    // Per-track Euclidean rhythm pattern (density determines how many of 16 steps trigger)
    bool triggerSequence[TRIGGER_SEQUENCE_LENGTH] = {false};
    
    // Per-track progression state
    uint8_t noteIndex = 0;
    bool upDownDirection = true;  // true = ascending, false = descending (for up-down mode)
    uint8_t lastNote = 0;  // For brownian mode
    
    // Per-track note scheduling
    uint32_t lastNoteTime = 0;
    
    // Per-track note-off scheduling
    uint64_t noteOffSample = 0;
    bool noteOffPending = false;
    uint8_t noteToTurnOff = 0;
    int8_t voiceToTurnOff = -1;
    
    // Inline helper to get density value (0-16)
    inline int GetDensityValue() {
        return static_cast<int>(density + 0.5f);
    }
    
    // Reset track state
    inline void Reset() {
        noteIndex = 0;
        upDownDirection = true;
        lastNote = 0;
        noteOffPending = false;
        noteToTurnOff = 0;
        voiceToTurnOff = -1;
        noteOffSample = 0;
    }
};

// Forward declarations for sequencer timing helpers (used inside struct)
inline void UpdateSequencerTiming();

// Forward declarations for track offset functions (used inside SequencerParams::Init)
inline float GetTrackOffset(int trackIdx);
inline void SetTrackOffset(int trackIdx, float normalizedValue);

// Sequencer parameters and state
struct SequencerParams {
    // Shared sequencer state
    bool sequencerMode = false;
    bool sequencerUsingInitialCapture = false;
    uint32_t envelopeDurationMs = 0;  // Calculated from attack+decay for latch mode
    
    // Shared density for trigger sequence generation (Euclidean rhythm)
    float density = 0.0f;  // 0.0f to 16.0f - used to generate trigger sequence pattern
    
    // Shared note pool - all tracks derive notes from this
    std::vector<uint8_t> sequencerNotes;  // Array of held notes for sequencer
    
    // Four independent sequencer tracks
    SequencerTrack tracks[4];
    
    // Clock and timing (shared across all tracks)
    int32_t clockBpm = CLOCK_BPM_DEFAULT;
    uint32_t lastClockTime = 0;
    uint32_t clockInterval = 0;
    bool clockEnabled = true;
    
    uint8_t currentSequenceStep = 0;
    uint32_t lastSequenceStepTime = 0;
    uint32_t sequenceStepInterval = 0;
    uint32_t sequenceStepIntervalSamples = 0;
    uint8_t triggerNote = 36;  // MIDI note for triggers (C2)
    uint8_t ccTriggerChannel = 12;
    uint8_t ccValueChannel = 15;
    
    uint64_t lastSequenceStepSample = 0;
    
    // Inline initialization (no function call overhead)
    void Init() {
        sequencerNotes.clear();
        clockInterval = static_cast<uint32_t>(60000 / (clockBpm * 24));
        lastClockTime = hw.seed.system.GetNow();
        UpdateSequencerTiming();
        lastSequenceStepSample = 0;
        
        // Initialize all tracks
        for (int i = 0; i < 4; i++) {
            tracks[i].Reset();
            // Initialize offset from UserState
            float offsetVal = GetTrackOffset(i);
            SetTrackOffset(i, offsetVal);
        }
        
        InitTriggerSequence();
    }
    
    inline void UpdateClockInterval() {
        clockInterval = static_cast<uint32_t>(60000 / (clockBpm * 24));
    }
};

SequencerParams sequencer;

inline void UpdateSequencerTimingMs()
{
    if (sequencer.clockBpm <= 0)
    {
        sequencer.clockBpm = CLOCK_BPM_MIN;
    }
    sequencer.sequenceStepInterval = static_cast<uint32_t>(60000.0f / (sequencer.clockBpm * 4.0f));
    sequencer.sequenceStepInterval = std::max<uint32_t>(1, sequencer.sequenceStepInterval);
}

inline void UpdateSequencerTimingSamples()
{
    float sampleRate = GetAudioSampleRate();
    if (sampleRate <= 0.0f)
    {
        sampleRate = 48000.0f;
    }
    float stepSamples = (sampleRate * 60.0f) / (static_cast<float>(sequencer.clockBpm) * 4.0f);
    sequencer.sequenceStepIntervalSamples = static_cast<uint32_t>(stepSamples);
    sequencer.sequenceStepIntervalSamples = std::max<uint32_t>(1, sequencer.sequenceStepIntervalSamples);
}

inline void UpdateSequencerTiming()
{
    UpdateSequencerTimingMs();
    UpdateSequencerTimingSamples();
}

inline uint32_t MsToSamples(float milliseconds)
{
    float sampleRate = GetAudioSampleRate();
    if (sampleRate <= 0.0f)
    {
        sampleRate = 48000.0f;
    }
    uint32_t samples = static_cast<uint32_t>((sampleRate * milliseconds) / 1000.0f);
    return std::max<uint32_t>(1, samples);
}

inline uint32_t ClampNoteOffDelaySamples(uint32_t desiredSamples)
{
    uint32_t guardSamples = MsToSamples(10.0f);
    guardSamples = std::max<uint32_t>(1, guardSamples);
    
    if (sequencer.sequenceStepIntervalSamples <= guardSamples + 1)
    {
        return std::max<uint32_t>(1, sequencer.sequenceStepIntervalSamples);
    }
    return std::max<uint32_t>(1, std::min(desiredSamples, sequencer.sequenceStepIntervalSamples - guardSamples));
}

inline uint32_t GetNoteOffDelaySamples(float noteLengthPercent)
{
    uint32_t desiredSamples = static_cast<uint32_t>(sequencer.sequenceStepIntervalSamples * noteLengthPercent);
    return ClampNoteOffDelaySamples(desiredSamples);
}


// Delay time helper (centralized) - implemented after sequencer declaration
inline float DelayTimeToMs(float normalizedTime, bool isShortMode) {
    if (isShortMode) {
        return LinearMap(normalizedTime, ParamConfig::DELAY_SHORT_MIN_MS, ParamConfig::DELAY_SHORT_MAX_MS);
    } else {
        // Long mode - BPM synced
        int index = static_cast<int>(normalizedTime * (ParamConfig::NUM_BEAT_MULTIPLIERS - 0.01f));
        index = std::max(0, std::min(ParamConfig::NUM_BEAT_MULTIPLIERS - 1, index));
        float beatMultiplier = ParamConfig::BEAT_MULTIPLIERS[index];
        float beatDuration = 60.0f / static_cast<float>(sequencer.clockBpm);
        return beatMultiplier * beatDuration * 1000.0f;  // Convert to ms
    }
}

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
    PARAM_PAN_WAVEFORM,
    PARAM_VOLUME,
    
    // OSC Panel
    PARAM_OSC_WAVEFORM,
    PARAM_OSC_MODE,
    PARAM_OSC_FM2_RATIO,
    PARAM_OSC_FM2_INDEX,
    PARAM_OSC_HARMONIC_IDX,
    PARAM_OSC_HARMONIC_DECAY,
    PARAM_OSC_HARMONIC_SKEW,
    
    // TUNING Panel
    PARAM_TUNING_INDEX,
    PARAM_TUNING_MIDI_ENABLE,
    PARAM_TUNING_BPM,  // BPM moved from SEQ panel
    
    // Sequencer Track 0 Panel
    PARAM_SEQ0_DENSITY,
    PARAM_SEQ0_ORDER,
    PARAM_SEQ0_CC_PROB,
    PARAM_SEQ0_OFFSET,
    
    // Sequencer Track 1 Panel
    PARAM_SEQ1_DENSITY,
    PARAM_SEQ1_ORDER,
    PARAM_SEQ1_CC_PROB,
    PARAM_SEQ1_OFFSET,
    
    // Sequencer Track 2 Panel
    PARAM_SEQ2_DENSITY,
    PARAM_SEQ2_ORDER,
    PARAM_SEQ2_CC_PROB,
    PARAM_SEQ2_OFFSET,
    
    // Sequencer Track 3 Panel
    PARAM_SEQ3_DENSITY,
    PARAM_SEQ3_ORDER,
    PARAM_SEQ3_CC_PROB,
    PARAM_SEQ3_OFFSET,
    
    // DELAY Panel
    PARAM_DELAY_MODE,
    PARAM_DELAY_TIME,
    PARAM_DELAY_DAMP,
    PARAM_DELAY_WETDRY,
    
    // MICROSOUND Panel
    PARAM_MICRO_PULSARET_LENGTH,
    PARAM_MICRO_PULSE_WIDTH,
    PARAM_MICRO_MODULATION,
    PARAM_MICRO_WETDRY,
    
    // BNRL (Binaural) Panel
    PARAM_BNRL_ENABLE,
    PARAM_BNRL_SPREAD,
    
    PARAM_NONE  // Used for unbound knobs
};

// UserState struct - single source of truth for all user-adjustable parameters
// All normalizedValues stored in normalized 0.0-1.0 range
struct UserState {
    // ADSR parameters (stored directly in milliseconds)
    float adsrAttackMs;        // Attack time in milliseconds (0.1ms - 1500ms)
    float adsrDecayReleaseMs;   // Decay/Release time in milliseconds (0.1ms - 1500ms)
    float adsrSustain;          // Sustain level 0.0-1.0
    float adsrMin;              // Minimum envelope level 0.0-1.0
    
    // MIXER parameters
    float panFreq;              // 0.0-1.0 maps to 0-10Hz
    float panAmp;               // 0.0-1.0 amplitude (0=centered, 1=full pan)
    float panWaveform;          // 0.0-1.0 waveform for pan LFO (same as oscWaveform: sine->tri->square->saw)
    float volume;               // 0.0-1.0 master volume
    
    // OSC parameters
    float oscWaveform;          // 0.0-1.0 (sine->tri->square->saw)
    float oscMode;              // 0.0-1.0 oscillator mode (0=Interpolated, 1=FM2, 2=Harmonic)
    float fm2Ratio;             // 0.0-1.0 maps to 0.125-8.0
    float fm2Index;             // 0.0-1.0 maps to 0.0-1.0
    float harmonicIdx;          // 0.0-1.0 maps to 1-16 as integers
    float harmonicDecay;        // 0.0-1.0 maps to 0-10 decay rate
    float harmonicSkew;         // 0.0-1.0 skew position (0=fundamental, 1=high harmonics)
    
    // TUNING parameters
    float tuningIndex;          // 0.0-1.0 maps to tuning preset index
    float tuningMidiEnable;     // 0.0-1.0 (>0.5 = enabled)
    float tuningBpm;            // 0.0-1.0 maps to CLOCK_BPM_MIN-CLOCK_BPM_MAX (moved from SEQ panel)
    
    // Per-track sequencer parameters (4 parameters per track: density, order, cc probability, offset)
    // Track 0
    float seq0Density;          // 0.0-1.0 maps to 0-16 triggers for track 0
    float seq0Order;            // 0.0-1.0 maps to order mode for track 0
    float seq0CcProb;           // 0.0-1.0 maps to 0-100% CC probability for track 0
    float seq0Offset;           // 0.0-1.0 maps to offset in semitones (-12, -5, 0, 5, 12)
    // Track 1
    float seq1Density;
    float seq1Order;
    float seq1CcProb;
    float seq1Offset;
    // Track 2
    float seq2Density;
    float seq2Order;
    float seq2CcProb;
    float seq2Offset;
    // Track 3
    float seq3Density;
    float seq3Order;
    float seq3CcProb;
    float seq3Offset;
    
    // Per-track CC inverse mode (one per track pair)
    bool seq0CcInverse;          // If true, second CC slot inverts the sequence
    bool seq1CcInverse;
    bool seq2CcInverse;
    bool seq3CcInverse;
    
    // Deprecated SEQUENCER parameters (kept for backward compatibility)
    float seqDensity;           // Deprecated - use per-track density
    float seqOrder;             // Deprecated - use per-track order
    float seqLength;            // Deprecated - no longer used
    float seqBpm;               // Deprecated - use tuningBpm
    float seqTrack0Density;     // Deprecated - use seq0Density
    float seqTrack1Density;     // Deprecated - use seq1Density
    float seqTrack2Density;     // Deprecated - use seq2Density
    float seqTrack3Density;     // Deprecated - use seq3Density
    
    // DELAY parameters
    float delayMode;           // 0.0-1.0 (short=0.0-0.5, long=0.5-1.0)
    float delayTime;           // 0.0-1.0 normalized delay time
    float delayDamp;           // 0.0-1.0 damping coefficient (pluck-like filter)
    float delayWetDry;         // 0.0-1.0 wet/dry mix
    
    // MICROSOUND parameters
    float microPulsaretLength;  // 0.0-1.0 maps to 1-100ms
    float microPulseWidth;      // 0.0-1.0 duty cycle
    float microModulation;      // 0.0-1.0 texture parameter
    float microWetDry;          // 0.0-1.0 wet/dry mix
    
    // BNRL (Binaural) parameters
    float bnrlEnable;           // 0.0-0.5 = off, 0.5-1.0 = on
    float bnrlSpread;           // 0.0-1.0 maps to 0-30 Hz
    
    // VOICE AMPLITUDE parameters (controlled via MIDI CC 100-103)
    float voiceAmplitudes[4];   // 0.0-1.0 amplitude for voices 0-3
    
    // Constructor with default normalizedValues
    float initialValue = 0.0f;
    UserState() :
        adsrAttackMs(initialValue),         // 0.1ms attack
        adsrDecayReleaseMs(initialValue), // 2.5s decay/release (2500ms)
        adsrSustain(initialValue),           // Full sustain
        adsrMin(initialValue),
        panFreq(initialValue),         // Default 0.2Hz
        panAmp(initialValue),           // Default full amplitude
        panWaveform(initialValue),      // Default sine wave for pan LFO
        volume(initialValue),           // Default 80% volume
        oscWaveform(initialValue),      // Default sine wave
        oscMode(initialValue),          // Default Interpolated oscillator
        fm2Ratio(initialValue),        // Default 1.0 ratio (0.26 normalizes to 1.0)
        fm2Index(initialValue),         // Default 0.5 index
        harmonicIdx(initialValue),      // Default harmonic index 1 (0.0 normalizes to 1)
        harmonicDecay(initialValue),    // Default decay rate
        harmonicSkew(initialValue),     // Default no skew (emphasize fundamental)
        tuningIndex(initialValue),      // Default 12-TET
        tuningMidiEnable(initialValue), // Default enabled
        tuningBpm((CLOCK_BPM_DEFAULT - CLOCK_BPM_MIN) / static_cast<float>(CLOCK_BPM_MAX - CLOCK_BPM_MIN)),  // Default 120 BPM normalized
        // Track 0 sequencer parameters
        seq0Density(initialValue),      // Default 0.0 (no triggers)
        seq0Order(initialValue),        // Default ascending
        seq0CcProb(initialValue),       // Default 0% CC probability
        seq0Offset(0.5f),               // Default 0 semitones (no change)
        // Track 1 sequencer parameters
        seq1Density(initialValue),
        seq1Order(initialValue),
        seq1CcProb(initialValue),
        seq1Offset(0.5f),               // Default 0 semitones (no change)
        // Track 2 sequencer parameters
        seq2Density(initialValue),
        seq2Order(initialValue),
        seq2CcProb(initialValue),
        seq2Offset(0.5f),               // Default 0 semitones (no change)
        // Track 3 sequencer parameters
        seq3Density(initialValue),
        seq3Order(initialValue),
        seq3CcProb(initialValue),
        seq3Offset(0.5f),               // Default 0 semitones (no change)
        // CC inverse mode (default to false - normal behavior)
        seq0CcInverse(false),
        seq1CcInverse(false),
        seq2CcInverse(false),
        seq3CcInverse(true),
        // Deprecated parameters (kept for backward compatibility)
        seqDensity(0.0f),
        seqOrder(initialValue),         // Default ascending
        seqLength(initialValue),        // Default 50% length
        seqBpm((CLOCK_BPM_DEFAULT - CLOCK_BPM_MIN) / static_cast<float>(CLOCK_BPM_MAX - CLOCK_BPM_MIN)),  // Default 120 BPM normalized
        seqTrack0Density(initialValue),  // Default 0.0 (no triggers)
        seqTrack1Density(initialValue),
        seqTrack2Density(initialValue),
        seqTrack3Density(initialValue),
        delayMode(initialValue),        // Default short mode
        delayTime(initialValue),        // Default 50% delay time
        delayDamp(initialValue),        // Default 30% damping (pluck-like filter)
        delayWetDry(initialValue),      // Default 30% wet mix
        microPulsaretLength(initialValue),  // Default 20ms (0.2 maps to ~20ms in 1-100ms range)
        microPulseWidth(initialValue),      // Default 50% duty cycle
        microModulation(initialValue),      // Default no modulation
        microWetDry(initialValue),          // Default 0% wet (fully dry - no microsound)
        bnrlEnable(initialValue),           // Default off
        bnrlSpread(initialValue),           // Default 0 Hz spread
        voiceAmplitudes{1.0f, 1.0f, 1.0f, 1.0f}  // Default full amplitude for all voices
    {}
};

// Global state instance
UserState appState;

// Helper functions to get track parameter values from appState
inline float GetTrackDensity(int trackIdx) {
    switch(trackIdx) {
        case 0: return appState.seq0Density;
        case 1: return appState.seq1Density;
        case 2: return appState.seq2Density;
        case 3: return appState.seq3Density;
        default: return 0.0f;
    }
}

inline float GetTrackOrder(int trackIdx) {
    switch(trackIdx) {
        case 0: return appState.seq0Order;
        case 1: return appState.seq1Order;
        case 2: return appState.seq2Order;
        case 3: return appState.seq3Order;
        default: return 0.0f;
    }
}

inline float GetTrackCcProb(int trackIdx) {
    switch(trackIdx) {
        case 0: return appState.seq0CcProb;
        case 1: return appState.seq1CcProb;
        case 2: return appState.seq2CcProb;
        case 3: return appState.seq3CcProb;
        default: return 0.0f;
    }
}

inline float GetTrackOffset(int trackIdx) {
    switch(trackIdx) {
        case 0: return appState.seq0Offset;
        case 1: return appState.seq1Offset;
        case 2: return appState.seq2Offset;
        case 3: return appState.seq3Offset;
        default: return 0.5f;
    }
}

// Helper function to map normalized value to SequencerOrderMode
inline SequencerOrderMode NormalizedToOrderMode(float normalizedValue) {
    if (normalizedValue < 0.1667f) return SEQ_ORDER_ASC;
    else if (normalizedValue < 0.3333f) return SEQ_ORDER_DESC;
    else if (normalizedValue < 0.5f) return SEQ_ORDER_UPD;
    else if (normalizedValue < 0.6667f) return SEQ_ORDER_FWD;
    else if (normalizedValue < 0.8333f) return SEQ_ORDER_RND;
    else return SEQ_ORDER_BRN;
}

// Helper function to update track density and regenerate trigger sequence
inline void SetTrackDensity(int trackIdx, float normalizedValue) {
    SequencerTrack& track = sequencer.tracks[trackIdx];
    track.density = normalizedValue * 16.0f;  // Map 0.0-1.0 to 0-16
    
    if (sequencer.sequencerMode) {
        int numTriggers = static_cast<int>(track.density + 0.5f);
        numTriggers = std::max(0, std::min(numTriggers, static_cast<int>(TRIGGER_SEQUENCE_LENGTH)));
        GenerateEuclideanRhythm(numTriggers, TRIGGER_SEQUENCE_LENGTH, track.triggerSequence);
    }
}

// Helper function to update track order mode (declared after SortSequencerNotes)
inline void SetTrackOrder(int trackIdx, float normalizedValue);

// Helper function to update track CC probability
inline void SetTrackCcProb(int trackIdx, float normalizedValue) {
    if (!sequencer.sequencerMode) {
        uint8_t prob = static_cast<uint8_t>(normalizedValue * 100.0f);
        prob = std::min(static_cast<uint8_t>(100), prob);
        int slotBase = trackIdx * 2;
        ccSlotProbabilities[slotBase] = prob;
        ccSlotProbabilities[slotBase + 1] = prob;
    }
}

// Helper function to update track offset
inline void SetTrackOffset(int trackIdx, float normalizedValue) {
    SequencerTrack& track = sequencer.tracks[trackIdx];
    
    // Map normalized value (0.0-1.0) to 7 discrete offset values in semitones: -12, -5, -4, 0, 4, 5, 12
    float offsetSemitones;
    if (normalizedValue < 0.142857f) {
        offsetSemitones = -12.0f;
    } else if (normalizedValue < 0.285714f) {
        offsetSemitones = -5.0f;
    } else if (normalizedValue < 0.428571f) {
        offsetSemitones = -4.0f;
    } else if (normalizedValue < 0.571429f) {
        offsetSemitones = 0.0f;
    } else if (normalizedValue < 0.714286f) {
        offsetSemitones = 4.0f;
    } else if (normalizedValue < 0.857143f) {
        offsetSemitones = 5.0f;
    } else {
        offsetSemitones = 12.0f;
    }
    
    track.offset = offsetSemitones;
    
    // Update appState
    switch(trackIdx) {
        case 0: appState.seq0Offset = normalizedValue; break;
        case 1: appState.seq1Offset = normalizedValue; break;
        case 2: appState.seq2Offset = normalizedValue; break;
        case 3: appState.seq3Offset = normalizedValue; break;
    }
}

// Function to update delay time calculation (called when parameters change)
void UpdateDelayTime() {
    float mode = appState.delayMode;
    float time = appState.delayTime;
    
    // Use centralized helper function
    float delayMs = DelayTimeToMs(time, mode < 0.5f);
    float delayTimeSamples = (delayMs / 1000.0f) * delaySampleRate;
    
    // Clamp delay time to valid range
    float maxDelay = static_cast<float>(MAX_DELAY_SAMPLES - 1);
    currentDelayTimeSamples = std::max(1.0f, std::min(delayTimeSamples, maxDelay));
    
    // For resampling delay, we don't set delay directly on the delay lines
    // Instead, we initialize read positions to the target delay time
    // The read positions will be updated smoothly in ApplyDelay()
    delayReadPosL = currentDelayTimeSamples;
    delayReadPosR = currentDelayTimeSamples;
}

// State accessor functions
float GetParamValue(ParamId paramId);
void SetParamValue(ParamId paramId, float normalizedValue);
float GetKnobValue(int panelIndex, int knobIndex);  // Helper to get knob normalizedValue from UserState

// Utility functions
float MidiNoteToFrequency(int8_t note, int8_t channel);

// Panel knob binding structure
struct PanelKnobBinding {
    ParamId knob1;
    ParamId knob2;
    ParamId knob3;
    ParamId knob4;
};

struct panelStruct
{
    const char*         name;
    char                id;
    const char*         input1Name;
    const char*         input2Name;
    const char*         input3Name;
    const char*         input4Name;
    PanelKnobBinding    bindings;       // New binding system
};

// Maximum number of panels - increase this if adding more panels
constexpr int MAX_PANELS = 16;

// Shared strings to reduce flash usage
static const char* SEQ_INPUT_NAMES[] = {"Dens", "Ord", "CC%", "Mult"};

panelStruct displayPanels[] = {
    { 
        name: "ENV",
        id: 'e',
        input1Name: "A", 
        input2Name: "D/R", 
        input3Name: "S",
        input4Name: "Min",
        bindings: {PARAM_ADSR_ATTACK, PARAM_ADSR_DECAY_RELEASE, PARAM_ADSR_SUSTAIN, PARAM_ADSR_MIN}
    },
    {
        name: "MIX",
        id: 'm',
        input1Name: "Freq",
        input2Name: "Amp",
        input3Name: "Wave",
        input4Name: "Vol",
        bindings: {PARAM_PAN_FREQ, PARAM_PAN_AMP, PARAM_PAN_WAVEFORM, PARAM_VOLUME}
    },
    {
        name: "OSC",
        id: 'o',
        input1Name: "Wav",
        input2Name: "",
        input3Name: "",
        input4Name: "Mode",
        bindings: {PARAM_OSC_WAVEFORM, PARAM_NONE, PARAM_NONE, PARAM_OSC_MODE}
    },
    {
        name: "TUNING",
        id: 't',
        input1Name: "T",
        input2Name: "",
        input3Name: "",
        input4Name: "BPM",
        bindings: {PARAM_TUNING_INDEX, PARAM_NONE, PARAM_TUNING_MIDI_ENABLE, PARAM_TUNING_BPM}
    },
    {
        name: "SEQ1",
        id: '0',
        input1Name: STR_DENS,
        input2Name: STR_ORD,
        input3Name: STR_CCP,
        input4Name: STR_OFFSET,
        bindings: {PARAM_SEQ0_DENSITY, PARAM_SEQ0_ORDER, PARAM_SEQ0_CC_PROB, PARAM_SEQ0_OFFSET}
    },
    {
        name: "SEQ2",
        id: '1',
        input1Name: STR_DENS,
        input2Name: STR_ORD,
        input3Name: STR_CCP,
        input4Name: STR_OFFSET,
        bindings: {PARAM_SEQ1_DENSITY, PARAM_SEQ1_ORDER, PARAM_SEQ1_CC_PROB, PARAM_SEQ1_OFFSET}
    },
    {
        name: "SEQ3",
        id: '2',
        input1Name: STR_DENS,
        input2Name: STR_ORD,
        input3Name: STR_CCP,
        input4Name: STR_OFFSET,
        bindings: {PARAM_SEQ2_DENSITY, PARAM_SEQ2_ORDER, PARAM_SEQ2_CC_PROB, PARAM_SEQ2_OFFSET}
    },
    {
        name: "SEQ4",
        id: '3',
        input1Name: STR_DENS,
        input2Name: STR_ORD,
        input3Name: STR_CCP,
        input4Name: STR_OFFSET,
        bindings: {PARAM_SEQ3_DENSITY, PARAM_SEQ3_ORDER, PARAM_SEQ3_CC_PROB, PARAM_SEQ3_OFFSET}
    },
    {
        name: "DLY",
        id: 'd',
        input1Name: "Mode",
        input2Name: "Time",
        input3Name: "Damp",
        input4Name: "Mix",
        bindings: {PARAM_DELAY_MODE, PARAM_DELAY_TIME, PARAM_DELAY_DAMP, PARAM_DELAY_WETDRY}
    },
    {
        name: "MICRO",
        id: 'g',
        input1Name: "PLen",
        input2Name: "PWid",
        input3Name: "Mod",
        input4Name: "Mix",
        bindings: {PARAM_MICRO_PULSARET_LENGTH, PARAM_MICRO_PULSE_WIDTH, 
                   PARAM_MICRO_MODULATION, PARAM_MICRO_WETDRY}
    },
    {
        name: "BNRL",
        id: 'b',
        input1Name: "On",
        input2Name: "Sprd",
        input3Name: "",
        input4Name: "",
        bindings: {PARAM_BNRL_ENABLE, PARAM_BNRL_SPREAD, PARAM_NONE, PARAM_NONE}
    },
    {
        name: "SAVE",
        id: 'p',
        input1Name: "",
        input2Name: "",
        input3Name: "",
        input4Name: "",
        bindings: {PARAM_NONE, PARAM_NONE, PARAM_NONE, PARAM_NONE}
    }
};

// Calculate panel count dynamically from array size
constexpr int panelModesCount = sizeof(displayPanels) / sizeof(displayPanels[0]);
static_assert(panelModesCount <= MAX_PANELS, "Too many panels! Increase MAX_PANELS");

panelStruct currentPanel;
int noteCount = 0;

float previousKnobState [4];
float smoothedKnobState[4] = {0.0f, 0.0f, 0.0f, 0.0f};
bool knobCaughtUp[4] = {false, false, false, false};  // Track if knob has caught up to stored knob normalizedValue

// Global knob normalizedValues storage - stores all knob positions for all panels (0.0-1.0 normalized)
// Size matches actual panel count to save memory
float knobValues[12][4] = {  // panelModesCount = 12
    {0.0f, 0.5f, 0.8f, 0.0f},  // Panel 0: ADSR
    {0.1f, 1.0f, 0.0f, 0.8f},  // Panel 1: MIXER
    {0.5f, 0.0f, 0.0f, 0.5f},  // Panel 2: OSC
    {0.9f, 0.0f, 1.0f, 0.186441f},  // Panel 3: TUNING (T, empty, MIDI, BPM) - Default 120 BPM normalized (110/590)
    {0.25f, 0.0f, 0.0f, 0.5f},  // Panel 4: SEQ0 (Dens, Ord, CC%, unused)
    {0.25f, 0.0f, 0.0f, 0.5f},  // Panel 5: SEQ1 (Dens, Ord, CC%, unused)
    {0.25f, 0.0f, 0.0f, 0.5f},  // Panel 6: SEQ2 (Dens, Ord, CC%, unused)
    {0.25f, 0.0f, 0.0f, 0.5f},  // Panel 7: SEQ3 (Dens, Ord, CC%, unused)
    {0.7f, 0.5f, 0.5f, 0.0f},  // Panel 8: DELAY (Mode, Time, Damp, Mix)
    {0.2f, 0.5f, 0.0f, 0.0f},  // Panel 9: MICRO (PLen, PWid, Mod, Mix)
    {0.0f, 0.0f, 0.0f, 0.0f},  // Panel 10: BNRL (On, Spread, unused, unused)
    {0.0f, 0.0f, 0.0f, 0.0f}   // Panel 11: PRESET
};
const float KNOB_CATCHUP_THRESHOLD = 0.05f;  // How close knob must be to catch up (5%)
constexpr float MICROSOUND_MIN_ACTIVE_MIX = 0.02f; // Keep microsound fully bypassed near 0%

struct voiceStruct
{
    int8_t note;
    int8_t velocity;
    // Removed allocationOrder - not used (voice stealing uses round-robin)
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
    bool      noteGate;
    bool      latchActive;
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
void      ApplyMicrosound(float* data);
void      ApplyDelay(float* data);
void      ApplyLimiter(float* data);
void      UpdateDelayTime();
void      UpdateOled();
// void      plucksApply();
void      InitPan(float samplerate);
void      SendPitchBend(uint8_t channel, int16_t bendValue);
void      ProcessCCSlots();
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



bool      sdCardInitialized = false;

constexpr float ADSR_SUSTAIN_FULL_LEVEL_ZONE_START   = 0.75f;
constexpr float ADSR_SUSTAIN_LATCH_ENABLE_THRESHOLD  = 0.90f;

bool  adsrLatchEnabled            = false;
bool  adsrSustainFullLevelLocked  = true;
float currentSustainLevel         = 1.0f;

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
        case PARAM_PAN_WAVEFORM:        return appState.panWaveform;
        case PARAM_VOLUME:              return appState.volume;
        
        // OSC Panel
        case PARAM_OSC_WAVEFORM:        return appState.oscWaveform;
        case PARAM_OSC_MODE:            return appState.oscMode;
        case PARAM_OSC_FM2_RATIO:       return appState.fm2Ratio;
        case PARAM_OSC_FM2_INDEX:       return appState.fm2Index;
        case PARAM_OSC_HARMONIC_IDX:    return appState.harmonicIdx;
        case PARAM_OSC_HARMONIC_DECAY:  return appState.harmonicDecay;
        case PARAM_OSC_HARMONIC_SKEW:   return appState.harmonicSkew;
        
        // TUNING Panel
        case PARAM_TUNING_INDEX:        return appState.tuningIndex;
        case PARAM_TUNING_MIDI_ENABLE:  return appState.tuningMidiEnable;
        case PARAM_TUNING_BPM:          return appState.tuningBpm;
        
        // Sequencer Track 0 Panel
        case PARAM_SEQ0_DENSITY:        return GetTrackDensity(0);
        case PARAM_SEQ0_ORDER:          return GetTrackOrder(0);
        case PARAM_SEQ0_CC_PROB:       return GetTrackCcProb(0);
        case PARAM_SEQ0_OFFSET:        return GetTrackOffset(0);
        
        // Sequencer Track 1 Panel
        case PARAM_SEQ1_DENSITY:        return GetTrackDensity(1);
        case PARAM_SEQ1_ORDER:          return GetTrackOrder(1);
        case PARAM_SEQ1_CC_PROB:        return GetTrackCcProb(1);
        case PARAM_SEQ1_OFFSET:        return GetTrackOffset(1);
        
        // Sequencer Track 2 Panel
        case PARAM_SEQ2_DENSITY:        return GetTrackDensity(2);
        case PARAM_SEQ2_ORDER:          return GetTrackOrder(2);
        case PARAM_SEQ2_CC_PROB:        return GetTrackCcProb(2);
        case PARAM_SEQ2_OFFSET:        return GetTrackOffset(2);
        
        // Sequencer Track 3 Panel
        case PARAM_SEQ3_DENSITY:        return GetTrackDensity(3);
        case PARAM_SEQ3_ORDER:          return GetTrackOrder(3);
        case PARAM_SEQ3_CC_PROB:        return GetTrackCcProb(3);
        case PARAM_SEQ3_OFFSET:        return GetTrackOffset(3);
        
        // DELAY Panel
        case PARAM_DELAY_MODE:         return appState.delayMode;
        case PARAM_DELAY_TIME:         return appState.delayTime;
        case PARAM_DELAY_DAMP:         return appState.delayDamp;
        case PARAM_DELAY_WETDRY:       return appState.delayWetDry;
        
        // MICROSOUND Panel
        case PARAM_MICRO_PULSARET_LENGTH: return appState.microPulsaretLength;
        case PARAM_MICRO_PULSE_WIDTH:     return appState.microPulseWidth;
        case PARAM_MICRO_MODULATION:      return appState.microModulation;
        case PARAM_MICRO_WETDRY:          return appState.microWetDry;
        
        case PARAM_NONE:
        default:                        return 0.0f;
    }
}

static inline float ComputeSustainLevelFromNormalized(float normalizedValue)
{
    if(normalizedValue < ADSR_SUSTAIN_FULL_LEVEL_ZONE_START)
    {
        return 0.01f * powf(100.0f, normalizedValue);
    }
    return 1.0f;
}

static inline bool ShouldEnableLatch(float normalizedValue)
{
    return normalizedValue >= ADSR_SUSTAIN_LATCH_ENABLE_THRESHOLD;
}

static inline bool IsFullLevelZone(float normalizedValue)
{
    return normalizedValue >= ADSR_SUSTAIN_FULL_LEVEL_ZONE_START;
}

// Update sequencer timing based on envelope duration for latch mode
static void UpdateSequencerEnvelopeTiming()
{
    // Update timing when latch is enabled in sequencer mode
    if (sequencer.sequencerMode && adsrLatchEnabled) {
        // Calculate envelope duration from attack + decay
        sequencer.envelopeDurationMs = static_cast<uint32_t>(appState.adsrAttackMs + appState.adsrDecayReleaseMs);
        
        // Clamp to reasonable range (minimum 10ms, maximum 10 seconds)
        const uint32_t minDuration = 10;
        const uint32_t maxDuration = 10000;
        sequencer.envelopeDurationMs = std::max(minDuration, std::min(maxDuration, sequencer.envelopeDurationMs));
        
        // Calculate BPM from envelope duration: BPM = 60000 / duration_ms
        // Clamp to reasonable BPM range (20-300 BPM)
        uint32_t calculatedBpm = 60000 / sequencer.envelopeDurationMs;
        const uint32_t minBpm = 20;
        const uint32_t maxBpm = 300;
        calculatedBpm = std::max(minBpm, std::min(maxBpm, calculatedBpm));
        
        // Update sequencer timing based on calculated BPM
        sequencer.clockBpm = static_cast<int32_t>(calculatedBpm);
        sequencer.clockInterval = static_cast<uint32_t>(60000 / (sequencer.clockBpm * 24));
        UpdateSequencerTiming();

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
                float attackMs = ExpMap(normalizedValue, ParamConfig::ADSR_MIN_MS, ParamConfig::ADSR_MAX_MS);
                appState.adsrAttackMs = std::max(ParamConfig::ADSR_MIN_MS, std::min(ParamConfig::ADSR_MAX_MS, attackMs));
                
                // Apply to all envelopes (convert ms to seconds)
                for (int i = 0; i < 4; i++) {
                    float attackTime = appState.adsrAttackMs / 1000.0f; // Convert ms to seconds
                    envelopes[i].env.SetTime(ADSR_SEG_ATTACK, attackTime);
                }
                
                // Update sequencer timing if in latch mode
                UpdateSequencerEnvelopeTiming();
            }
            break;
            
        case PARAM_ADSR_DECAY_RELEASE:
            {
                // Convert normalized normalizedValue (0.0-1.0) to milliseconds
                // Use exponential mapping for better control over short times
                float decayMs = ExpMap(normalizedValue, ParamConfig::ADSR_MIN_MS, ParamConfig::ADSR_MAX_MS);
                appState.adsrDecayReleaseMs = std::max(ParamConfig::ADSR_MIN_MS, std::min(ParamConfig::ADSR_MAX_MS, decayMs));
                
                // Apply to all envelopes (convert ms to seconds)
                for (int i = 0; i < 4; i++) {
                    float decayTime = appState.adsrDecayReleaseMs / 1000.0f; // Convert ms to seconds
                    envelopes[i].env.SetTime(ADSR_SEG_DECAY, decayTime);
                    envelopes[i].env.SetTime(ADSR_SEG_RELEASE, decayTime);
                }
                
                // Update sequencer timing if in latch mode
                UpdateSequencerEnvelopeTiming();
            }
            break;
            
        case PARAM_ADSR_SUSTAIN:
            {
                float clampedValue = std::max(0.0f, std::min(1.0f, normalizedValue));
                appState.adsrSustain = clampedValue;

                adsrLatchEnabled = ShouldEnableLatch(clampedValue);
                adsrSustainFullLevelLocked = IsFullLevelZone(clampedValue);
                currentSustainLevel = ComputeSustainLevelFromNormalized(clampedValue);

                // Apply to all envelopes and update latch state
                for (int i = 0; i < 4; i++) {
                    envelopes[i].env.SetSustainLevel(currentSustainLevel);

                    if (adsrLatchEnabled && envelopes[i].noteGate) {
                        envelopes[i].latchActive = true;
                    } else if (!adsrLatchEnabled && envelopes[i].latchActive) {
                        envelopes[i].latchActive = false;
                        if (envelopes[i].noteGate) {
                            envelopes[i].gate = true;
                        }
                    }
                }
            }
            break;
            
        case PARAM_ADSR_MIN:
            appState.adsrMin = normalizedValue;
            voicesMinLevel = normalizedValue;
            break;
        
        // MIXER Panel
        case PARAM_PAN_FREQ:
            appState.panFreq = normalizedValue;
            panFreq = LinearMap(normalizedValue, 0.0f, ParamConfig::PAN_FREQ_MAX_HZ);
            panLfo.SetFreq(panFreq);
            break;
            
        case PARAM_PAN_AMP:
            appState.panAmp = normalizedValue;
            panAmp = normalizedValue;
            break;
            
        case PARAM_PAN_WAVEFORM:
            appState.panWaveform = normalizedValue;
            panLfo.SetWaveformParam(normalizedValue);
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
            {
                float oldMode = appState.oscMode;
                appState.oscMode = normalizedValue;
                
                // Reset phase tracking when switching to/from Interpolated mode to prevent stale values
                int newMode = static_cast<int>(normalizedValue * 2.99f);  // 0-2: Interpolated, FM2, Harmonic
                int oldModeInt = static_cast<int>(oldMode * 2.99f);
                
                // Reset catch-up state for knobs 0-2 when mode changes
                // The same physical knobs control different parameters in different modes
                if (newMode != oldModeInt) {
                    for (int i = 0; i < 3; i++) {
                        knobCaughtUp[i] = false;
                    }
                }
                
                if ((newMode == 0 && oldModeInt != 0) || (newMode != 0 && oldModeInt == 0)) {
                    // Switching to/from Interpolated mode - reset phase tracking
                    for (int i = 0; i < 2; i++) {
                        externalPhaseTrack[i] = 0.0f;
                        externalPhasePrev[i] = 0.0f;
                        externalPhaseSmooth[i] = 0.0f;
                        inputPeakAmplitude[i] = 1.0f;  // Reset peak tracking
                        normalizedInputPrev[i] = 0.0f;  // Reset smoothing
                    }
                }
            }
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
                // FM index is now controlled by envelope amplitude per voice
                // This parameter is kept for UI display purposes but doesn't control the index
                // Clamp to 0 when below 0.1 to prevent unwanted modulation
                appState.fm2Index = (normalizedValue < 0.1f) ? 0.0f : normalizedValue;
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
        
        case PARAM_TUNING_BPM:
            {
                // In latch mode the envelope timing drives the sequencer, so ignore BPM knob input
                if (adsrLatchEnabled) {
                    float normalizedBpm = static_cast<float>(sequencer.clockBpm - CLOCK_BPM_MIN)
                                           / (CLOCK_BPM_MAX - CLOCK_BPM_MIN);
                    normalizedBpm = std::max(0.0f, std::min(1.0f, normalizedBpm));
                    appState.tuningBpm = normalizedBpm;
                    break;
                }

                // Calculate BPM from normalized value
                int32_t newBpm = CLOCK_BPM_MIN + static_cast<int32_t>(normalizedValue * (CLOCK_BPM_MAX - CLOCK_BPM_MIN));
                
                // Quantize to nearest multiple of 5
                newBpm = ((newBpm + 2) / 5) * 5;
                newBpm = std::max(CLOCK_BPM_MIN, std::min(CLOCK_BPM_MAX, newBpm));
                
                // Update normalized value to match quantized BPM so display reflects actual value
                appState.tuningBpm = static_cast<float>(newBpm - CLOCK_BPM_MIN) / (CLOCK_BPM_MAX - CLOCK_BPM_MIN);
                
                if (newBpm != sequencer.clockBpm) {
                    sequencer.clockBpm = newBpm;
                    sequencer.clockInterval = static_cast<uint32_t>(60000 / (sequencer.clockBpm * 24));
                    UpdateSequencerTiming();
                    // Update delay time if in BPM-sync mode
                    UpdateDelayTime();
                }
            }
            break;
        
        // Sequencer Track 0 Panel
        case PARAM_SEQ0_DENSITY:
            appState.seq0Density = normalizedValue;
            SetTrackDensity(0, normalizedValue);
            break;
            
        case PARAM_SEQ0_ORDER:
            appState.seq0Order = normalizedValue;
            SetTrackOrder(0, normalizedValue);
            break;
            
        case PARAM_SEQ0_CC_PROB:
            appState.seq0CcProb = normalizedValue;
            SetTrackCcProb(0, normalizedValue);
            break;
            
        case PARAM_SEQ0_OFFSET:
            appState.seq0Offset = normalizedValue;
            SetTrackOffset(0, normalizedValue);
            break;
        
        // Sequencer Track 1 Panel
        case PARAM_SEQ1_DENSITY:
            appState.seq1Density = normalizedValue;
            SetTrackDensity(1, normalizedValue);
            break;
            
        case PARAM_SEQ1_ORDER:
            appState.seq1Order = normalizedValue;
            SetTrackOrder(1, normalizedValue);
            break;
            
        case PARAM_SEQ1_CC_PROB:
            appState.seq1CcProb = normalizedValue;
            SetTrackCcProb(1, normalizedValue);
            break;
            
        case PARAM_SEQ1_OFFSET:
            appState.seq1Offset = normalizedValue;
            SetTrackOffset(1, normalizedValue);
            break;
        
        // Sequencer Track 2 Panel
        case PARAM_SEQ2_DENSITY:
            appState.seq2Density = normalizedValue;
            SetTrackDensity(2, normalizedValue);
            break;
            
        case PARAM_SEQ2_ORDER:
            appState.seq2Order = normalizedValue;
            SetTrackOrder(2, normalizedValue);
            break;
            
        case PARAM_SEQ2_CC_PROB:
            appState.seq2CcProb = normalizedValue;
            SetTrackCcProb(2, normalizedValue);
            break;
            
        case PARAM_SEQ2_OFFSET:
            appState.seq2Offset = normalizedValue;
            SetTrackOffset(2, normalizedValue);
            break;
        
        // Sequencer Track 3 Panel
        case PARAM_SEQ3_DENSITY:
            appState.seq3Density = normalizedValue;
            SetTrackDensity(3, normalizedValue);
            break;
            
        case PARAM_SEQ3_ORDER:
            appState.seq3Order = normalizedValue;
            SetTrackOrder(3, normalizedValue);
            break;
            
        case PARAM_SEQ3_CC_PROB:
            appState.seq3CcProb = normalizedValue;
            SetTrackCcProb(3, normalizedValue);
            break;
            
        case PARAM_SEQ3_OFFSET:
            appState.seq3Offset = normalizedValue;
            SetTrackOffset(3, normalizedValue);
            break;
            
        // DELAY Panel
        case PARAM_DELAY_MODE:
            appState.delayMode = normalizedValue;
            UpdateDelayTime();  // Recalculate delay time when mode changes
            break;
            
        case PARAM_DELAY_TIME:
            appState.delayTime = normalizedValue;
            UpdateDelayTime();  // Recalculate delay time when time changes
            break;
            
        case PARAM_DELAY_DAMP:
            appState.delayDamp = normalizedValue;
            // Damping is read in ApplyDelay() - no action needed here
            break;
            
        case PARAM_DELAY_WETDRY:
            appState.delayWetDry = normalizedValue;
            // Wet/dry mix is read in ApplyDelay() - no action needed here
            break;
        
        // MICROSOUND Panel
        case PARAM_MICRO_PULSARET_LENGTH:
            appState.microPulsaretLength = normalizedValue;
            // Map to 2-30ms range for tight, responsive grains
            pulsarSynth.SetPulsaretLength(2.0f + normalizedValue * 28.0f);
            break;
            
        case PARAM_MICRO_PULSE_WIDTH:
            appState.microPulseWidth = normalizedValue;
            pulsarSynth.SetPulseWidth(normalizedValue);
            break;
            
        case PARAM_MICRO_MODULATION:
            appState.microModulation = normalizedValue;
            pulsarSynth.SetModulation(normalizedValue);
            break;
            
        case PARAM_MICRO_WETDRY:
            appState.microWetDry = (normalizedValue < MICROSOUND_MIN_ACTIVE_MIX) ? 0.0f : normalizedValue;
            // Wet/dry mixing now happens in ApplyMicrosound, not in PulsarSynth
            break;
        
        // BNRL (Binaural) Panel
        case PARAM_BNRL_ENABLE:
            appState.bnrlEnable = normalizedValue;
            binauralEnabled = (normalizedValue >= 0.5f);
            break;
            
        case PARAM_BNRL_SPREAD:
            appState.bnrlSpread = normalizedValue;
            binauralSpreadHz = normalizedValue * 30.0f;  // 0-30 Hz range
            break;
        
        case PARAM_NONE:
        default:
            break;
    }
}

// Helper to get knob normalizedValue from UserState via panel bindings
float GetKnobValue(int panelIndex, int knobIndex)
{
    if (panelIndex < 0 || panelIndex >= panelModesCount || knobIndex < 0 || knobIndex >= 4) {
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
    float L1, R1, R2, L2, L3, R3, L4, R4;
    
    // Update pan phase manually to maintain control
    panPhase += 2.0f * M_PI * panFreq / hw.AudioSampleRate();
    if (panPhase >= 2.0f * M_PI) {
        panPhase -= 2.0f * M_PI;
    }
    
    // Convert pan phase from 0-2π to 0-1 range for LFO
    float lfoPhase = panPhase / (2.0f * M_PI);
    
    // Get LFO outputs for each voice with fixed phase offsets
    // Voices are evenly distributed: 0°, 90°, 180°, 270°
    // This ensures voices 0 and 2 are opposite (hard left/right at some point in LFO cycle)
    // and voices 1 and 3 are also opposite
    // Scale by panAmp: 0 = all centered, 1 = full panning effect
    float pan0 = panLfo.ProcessAtPhase(lfoPhase) * panAmp;       // Voice 0: 0° (base phase)
    float pan1 = panLfo.ProcessAtPhase(lfoPhase + 0.25f) * panAmp;  // Voice 1: +90°
    float pan2 = panLfo.ProcessAtPhase(lfoPhase + 0.5f) * panAmp;   // Voice 2: +180°
    float pan3 = panLfo.ProcessAtPhase(lfoPhase + 0.75f) * panAmp;  // Voice 3: +270°
    
    // Store LFO output for CV output (before amplitude scaling)
    float panOutput = panLfo.ProcessAtPhase(lfoPhase);
    
    PanEqualPowerStereo(pan0, data[0], &L1, &R1);
    PanEqualPowerStereo(pan1, data[1], &L2, &R2);
    PanEqualPowerStereo(pan2, data[2], &L3, &R3);
    PanEqualPowerStereo(pan3, data[3], &L4, &R4);

    // Sum all voices (volume control moved to crossfader)
    data[0] = L1 + L2 + L3 + L4;
    data[1] = R1 + R2 + R3 + R4;
    
    // Process binaural oscillators with opposite panning (no trig calls - uses LUTs)
    if (binauralEnabled) {
        static float sampleRateInv = 0.0f;
        if (sampleRateInv == 0.0f) sampleRateInv = 1.0f / hw.AudioSampleRate();
        
        float binL = 0.0f, binR = 0.0f;
        float panVals[4] = {-pan0, -pan1, -pan2, -pan3};  // Opposite pan values
        
        for (int i = 0; i < 4; i++) {
            float envVal = std::max(envelopes[i].envSig, voicesMinLevel);
            // Fast sine using lookup table
            float osc = fastSin(binauralPhase[i]) * envVal * 0.5f;
            binauralPhase[i] += binauralFreqHz[i] * sampleRateInv;
            if (binauralPhase[i] >= 1.0f) binauralPhase[i] -= 1.0f;
            
            // Fast equal power panning using lookup tables
            float l, r;
            fastPanEqualPower(panVals[i], osc, &l, &r);
            binL += l;
            binR += r;
        }
        
        data[0] += binL;
        data[1] += binR;
    }
    
    hw.seed.dac.WriteValue(DacHandle::Channel::ONE, ((panOutput + 1.0f) / 2.0f) * 4095);
}

void ApplyCrossfader(float* processedLeft, float* processedRight, 
                     float externalLeft, float externalRight) {
    // Crossfader: volume parameter controls mix between processed output and external audio
    // volume = 1.0: processed output at full, external audio at 0%
    // volume = 0.5: processed output at 50%, external audio at 50%
    // volume = 0.0: processed output at 0%, external audio at full
    float volume = appState.volume;
    *processedLeft = *processedLeft * volume + externalLeft * (1.0f - volume);
    *processedRight = *processedRight * volume + externalRight * (1.0f - volume);
}

void ApplyMicrosound(float* data) {
    // Get wet/dry parameter
    float wetDry = appState.microWetDry;
    
    // Skip ALL processing if wet/dry is 0 (fully dry)
    // This ensures no interference with the normal signal path
    if (wetDry <= MICROSOUND_MIN_ACTIVE_MIX) {
        return;  // Pass through unchanged
    }
    
    // Write current audio to microsound buffer (only when active)
    microsoundBufferL.Write(data[0]);
    microsoundBufferR.Write(data[1]);
    
    // Update microsound frequency to track the most recently played note
    // Uses currentNote which is updated by the Intellijel handler
    if (currentNote > 0) {
        float microsoundFreq = MidiNoteToFrequency(currentNote, 1);
        pulsarSynth.SetFrequency(microsoundFreq);
        microsoundNote = currentNote;
    }
    
    float microL = 0.0f;
    float microR = 0.0f;
    
    // Process microsound by reading from dedicated microsound buffers
    pulsarSynth.Process(microsoundBufferL, microsoundBufferR, microL, microR);
    
    // Check for invalid values
    if (!std::isfinite(microL)) microL = 0.0f;
    if (!std::isfinite(microR)) microR = 0.0f;
    
    // Apply moderate gain (1.0x - same as input), then clip
    microL *= 1.0f;
    microR *= 1.0f;
    microL = std::max(-1.0f, std::min(microL, 1.0f));
    microR = std::max(-1.0f, std::min(microR, 1.0f));
    
    // Apply wet/dry mixing
    float dryL = data[0];
    float dryR = data[1];
    data[0] = dryL * (1.0f - wetDry) + microL * wetDry;
    data[1] = dryR * (1.0f - wetDry) + microR * wetDry;
}

void ApplyDelay(float* data) {
    // Read delay parameters from UserState
    float dampCoeff = appState.delayDamp;  // Damping coefficient (0.0-1.0)
    float wetDry = appState.delayWetDry;
    
    // Use damping coefficient to derive feedback amount
    // Higher damping = lower feedback (more pluck-like decay)
    float feedback = 1.0f - (dampCoeff * 0.65f);
    
    // Smoothly move read positions toward target delay time
    float targetDelay = currentDelayTimeSamples;
    float smoothingFactor = 0.01f;
    
    delayReadPosL += (targetDelay - delayReadPosL) * smoothingFactor;
    delayReadPosR += (targetDelay - delayReadPosR) * smoothingFactor;
    
    // Clamp read positions to valid range
    float maxDelay = static_cast<float>(MAX_DELAY_SAMPLES - 1);
    delayReadPosL = std::max(1.0f, std::min(delayReadPosL, maxDelay));
    delayReadPosR = std::max(1.0f, std::min(delayReadPosR, maxDelay));
    
    // Read from delay lines using fractional positions
    float leftDelayed = delayL.ReadHermite(delayReadPosL);
    float rightDelayed = delayR.ReadHermite(delayReadPosR);
    
    // Safety check for invalid values
    if (!std::isfinite(leftDelayed)) leftDelayed = 0.0f;
    if (!std::isfinite(rightDelayed)) rightDelayed = 0.0f;
    
    // Apply one-pole lowpass filter in feedback path (pluck-like damping)
    // IMPORTANT: Use separate filter states for left and right channels
    float filterCoeff = dampCoeff * 0.9f;
    float filteredL = leftDelayed * (1.0f - filterCoeff) + delayFilterStateL * filterCoeff;
    float filteredR = rightDelayed * (1.0f - filterCoeff) + delayFilterStateR * filterCoeff;
    delayFilterStateL = filteredL;  // Update left filter state
    delayFilterStateR = filteredR;  // Update right filter state
    
    // Process left channel
    float leftIn = data[0];
    float leftFeedback = leftIn + (filteredL * feedback);
    delayL.Write(leftFeedback);
    
    // Process right channel
    float rightIn = data[1];
    float rightFeedback = rightIn + (filteredR * feedback);
    delayR.Write(rightFeedback);
    
    // Mix wet/dry
    data[0] = (leftIn * (1.0f - wetDry)) + (filteredL * wetDry);
    data[1] = (rightIn * (1.0f - wetDry)) + (filteredR * wetDry);
    
    // Final safety check
    if (!std::isfinite(data[0])) data[0] = 0.0f;
    if (!std::isfinite(data[1])) data[1] = 0.0f;
}

void ApplyLimiter(float* data) {
    // Process limiter on stereo channels (left and right)
    // The limiter's ProcessBlock works on buffers, so we'll process sample-by-sample
    // using a simple peak-tracking approach similar to the limiter's internal logic
    
    static float limiterPeakL = 0.5f;
    static float limiterPeakR = 0.5f;
    
    // Pre-gain (can be adjusted if needed, 1.0 = no pre-gain)
    float preGain = 0.45f;
    
    // Process left channel
    float leftPre = data[0] * preGain;
    float leftPeak = fabsf(leftPre);
    // SLOPE: smooth peak tracking (attack: 0.05, release: 0.00002)
    float leftError = leftPeak - limiterPeakL;
    limiterPeakL += (leftError > 0 ? 0.05f : 0.00002f) * leftError;
    float leftGain = (limiterPeakL <= 1.0f ? 1.0f : 1.0f / limiterPeakL);
    data[0] = SoftLimit(leftPre * leftGain * 0.65f);
    
    // Process right channel
    float rightPre = data[1] * preGain;
    float rightPeak = fabsf(rightPre);
    // SLOPE: smooth peak tracking (attack: 0.05, release: 0.00002)
    float rightError = rightPeak - limiterPeakR;
    limiterPeakR += (rightError > 0 ? 0.05f : 0.00002f) * rightError;
    float rightGain = (limiterPeakR <= 1.0f ? 1.0f : 1.0f / limiterPeakR);
    data[1] = SoftLimit(rightPre * rightGain * 0.65f);
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

// Convert Hz detuning to cents deviation for MIDI pitch bend
// Returns negative cents when detuning down (lower frequency)
float HzDetuningToCents(float baseFreq, float detuneHz)
{
    if (baseFreq <= 0.0f) return 0.0f;
    float detunedFreq = baseFreq - detuneHz;
    if (detunedFreq <= 0.0f) return 0.0f;
    return 1200.0f * log2f(detunedFreq / baseFreq);
}

// Process oscillator based on selected mode
// externalPhase: optional external phase value (0-1 range) for InterpolatedOscillator
float ProcessOscillator(int voiceIndex, float externalPhase = -1.0f) {
    int mode = static_cast<int>(appState.oscMode * 2.99f); // 0-2: Interpolated, FM2, Harmonic
    mode = std::max(0, std::min(2, mode)); // Clamp to 0-2
    
    switch(mode) {
        case 0: // Interpolated
            if (externalPhase >= 0.0f) {
                // Use external phase to address the wavetable
                return voiceInterpOsc[voiceIndex].ProcessAtPhase(externalPhase);
            } else {
                return voiceInterpOsc[voiceIndex].Process();
            }
        case 1: // FM2
            return voiceFm2Osc[voiceIndex].Process();
        case 2: // Harmonic
            return voiceHarmonicOsc[voiceIndex].Process();
        default:
            if (externalPhase >= 0.0f) {
                return voiceInterpOsc[voiceIndex].ProcessAtPhase(externalPhase);
            } else {
                return voiceInterpOsc[voiceIndex].Process();
            }
    }
}

void SetOscillatorFrequency(int voiceIndex, float baseFreq) {
    // Apply binaural detuning: main oscillator DOWN, binaural oscillator UP
    float detuneHz = binauralEnabled ? (binauralSpreadHz / 2.0f) : 0.0f;
    float mainFreq = baseFreq - detuneHz;
    float binFreq = baseFreq + detuneHz;
    
    // Set frequency for all main oscillator types so mode switching doesn't lose the pitch
    voiceInterpOsc[voiceIndex].SetFreq(mainFreq);
    voiceFm2Osc[voiceIndex].SetFrequency(mainFreq);
    voiceHarmonicOsc[voiceIndex].SetFreq(mainFreq);
    // Update internal phase oscillator frequency (used for phase addressing in voices 1 and 3)
    internalPhaseOsc[voiceIndex].SetFreq(mainFreq);
    
    // Set binaural oscillator frequency (detuned UP) - simple phase accumulator
    binauralFreqHz[voiceIndex] = binFreq;
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
        // Apply voice amplitude multiplier (from MIDI CC 100-103)
        data[i] = data[i] * envVal * appState.voiceAmplitudes[i];

        if (envVal > envMax)
        {
            envMax = envVal;
        }
    }

    int16_t vactrolOffset = 300; // this is to bias the Intellijel vactrol
    int16_t outputMaxEnvelope = vactrolOffset + envMax * (4095 - vactrolOffset);
    hw.seed.dac.WriteValue(DacHandle::Channel::TWO, outputMaxEnvelope);
}


void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    ProcessControls();
    
    // Process envelopes at audio rate for consistent timing
    for(int j = 0; j < 4; j++)
    {
        if(envelopes[j].latchActive)
        {
            // Check if we should disable latch (note released or mode changed)
            if(!adsrLatchEnabled || !envelopes[j].noteGate)
            {
                envelopes[j].latchActive = false;
                if(!envelopes[j].noteGate)
                {
                    envelopes[j].gate = false;
                }
            }
            else
            {
                uint8_t segment = envelopes[j].env.GetCurrentSegment();
                if(envelopes[j].gate)
                {
                    if(segment == ADSR_SEG_DECAY)
                    {
                        envelopes[j].gate = false;
                    }
                }
                else
                {
                    if(segment == ADSR_SEG_IDLE)
                    {
                        // Self-loop mode: retrigger self
                        envelopes[j].gate = true;
                    }
                }
            }
        }

        envelopes[j].envSig = envelopes[j].env.Process(envelopes[j].gate);
    }

    // Update FM index for each voice based on envelope amplitude
    // Scale envelope amplitude by the index parameter value
    for (int j = 0; j < 4; j++) {
        // Multiply envelope amplitude (0.0-1.0) by index parameter to get final FM index
        float index = envelopes[j].envSig * appState.fm2Index * 1;
        voiceFm2Osc[j].SetIndex(index);
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
        // Determine mode first to know how to initialize results
        int mode = static_cast<int>(appState.oscMode * 2.99f);  // 0-2: Interpolated, FM2, Harmonic
        mode = std::max(0, std::min(2, mode));
        
        for (size_t j = 0; j < 4; j++)
        {
            // In Interpolated mode, voices 0 and 2 use oscillator output, not input
            // Initialize them to zero to prevent input signal bleed
            if (mode == 0 && (j == 0 || j == 2)) {
                results[j] = 0.0f;  // Will be replaced with oscillator output
            } else {
                results[j] = in[j][i];  // Other voices use input directly
            }
        }

        // Process internal phase oscillators to advance their phase
        // These are used for phase addressing in voices 1 and 3
        for (int ch = 0; ch < 4; ch++) {
            internalPhaseOsc[ch].Process();  // Advance phase, output is discarded
        }
        
        // Always compute oscillator outputs for all voices
        // For InterpolatedOscillator: use external audio inputs or internal oscillators to address phase
        // Mode is already calculated above, reuse it
        for (int ch = 0; ch < 4; ch++) {
            float externalPhase = -1.0f;  // Default: no external phase
            
            // Only use external phase for InterpolatedOscillator mode (mode 0)
            // Mode variable is already set from initialization loop above
            
            if (mode == 0) {  // Interpolated mode
                if (ch == 0 || ch == 2) {
                // if (false) {
                    // Voices 0 and 2: use external audio inputs with phase unwrapping
                    float audioInput = 0.0f;
                    int trackIdx = (ch == 0) ? 0 : 1;  // Index into externalPhaseTrack array
                    
                    if (ch == 0) {
                        audioInput = in[0][i];
                    } else {  // ch == 2
                        audioInput = in[2][i];
                    }
                    
                    // Normalize input to full -1 to 1 range based on detected peak amplitude
                    // This ensures we always use the full 0-1 phase range regardless of input level
                    // Track peak amplitude with adaptive attack/decay to handle changing input levels
                    float absInput = fabsf(audioInput);
                    float attackAlpha = 0.01f;   // Faster attack when input exceeds peak
                    float decayAlpha = 0.0001f;  // Very slow decay to track decreasing levels
                    
                    if (absInput > inputPeakAmplitude[trackIdx]) {
                        // Fast attack when input exceeds current peak
                        inputPeakAmplitude[trackIdx] = inputPeakAmplitude[trackIdx] * (1.0f - attackAlpha) + absInput * attackAlpha;
                    } else {
                        // Slow decay to track decreasing input levels
                        inputPeakAmplitude[trackIdx] = inputPeakAmplitude[trackIdx] * (1.0f - decayAlpha) + absInput * decayAlpha;
                    }
                    
                    // Ensure minimum peak amplitude to avoid division issues
                    if (inputPeakAmplitude[trackIdx] < 0.001f) {
                        inputPeakAmplitude[trackIdx] = 0.001f;
                    }
                    
                    // Normalize input to -1 to 1 range based on detected peak
                    float normalizedInput = audioInput / (0.98f *inputPeakAmplitude[trackIdx]);
                    
                    // Clamp normalized input to prevent overshoot (keep away from edges to avoid clipping)
                    normalizedInput = std::max(-1.0f, std::min(1.0f, normalizedInput));
                    
                    // Apply light smoothing to reduce high-frequency artifacts
                    // This helps reduce aliasing and distortion from rapid phase changes
                    float smoothingFactor = 0.95f;  // Light smoothing (95% previous, 5% new)
                    normalizedInput = normalizedInputPrev[trackIdx] * smoothingFactor + normalizedInput * (1.0f - smoothingFactor);
                    normalizedInputPrev[trackIdx] = normalizedInput;
                    
                    // Map from -1 to 1 range to 0 to 1 range (phase range)
                    float mappedPhase = (normalizedInput + 1.0f) * 0.5f;
                    
                    // Normalize phase exactly like ProcessAtPhase does internally
                    while (mappedPhase >= 1.0f) mappedPhase -= 1.0f;
                    while (mappedPhase < 0.0f) mappedPhase += 1.0f;
                    
                    // Use mapped phase directly for wavetable lookup
                    externalPhase = mappedPhase;
                } else {
                    // Voices 1 and 3: use internal oscillator phase
                    externalPhase = internalPhaseOsc[ch].GetPhase();
                }
            }

            // externalPhase = 0.25f + externalPhase / 2.0f;
            
            oscOutputs[ch] = ProcessOscillator(ch, externalPhase);
        }
        
        // Mode is already calculated above, reuse it
        // For voices 0 and 2: use oscillator output when in Interpolated mode (mode 0)
        // For voices 1 and 3: always mix into stereo output if useInternalOscillators is enabled
        for (int ch = 0; ch < 4; ch++) {
            if (ch == 0 || ch == 2) {
                // Voices 0 and 2: use oscillator output in Interpolated mode
                if (mode == 0) {
                    results[ch] = oscOutputs[ch];
                } else if (useInternalOscillators[ch]) {
                    results[ch] = oscOutputs[ch];
                }
            } else {
                // Voices 1 and 3: use oscillator output if useInternalOscillators is enabled
                if (useInternalOscillators[ch]) {
                    results[ch] = oscOutputs[ch];
                }
            }
        }

        // Panel 1 - Envelope/Gain
        ApplyVCAs(results);

        // Panel 2 - Panning and mixing
        ApplyPanning(results);

        // Panel 4 - Microsound (reads from separate microsound buffer, mixes into results)
        ApplyMicrosound(results);

        // Panel 5 - Delay effect (independent from microsound)
        ApplyDelay(results);

        // Limiter - final stage to prevent clipping
        ApplyLimiter(results);

        // Crossfader: mix processed output with external audio from inputs 1 and 3
        // Input 1 goes to left channel, input 3 goes to right channel
        float processedLeft = results[0];
        float processedRight = results[1];
        float externalLeft = in[1][i];
        float externalRight = in[3][i];
        ApplyCrossfader(&processedLeft, &processedRight, externalLeft, externalRight);
        
        out[0][i] = processedLeft;
        out[1][i] = processedRight;
        
        // Output modulator signals for voices 0 and 2 to audio outputs 2 and 3
        // Only output modulator when in FM mode (mode == 1)
        float output2 = 0.0f;
        float output3 = 0.0f;
        
        if (mode == 1) {
            // Get modulator output from FM2 oscillators for voices 0 and 2
            if (envelopes[0].envSig > 0.0f) {
                output2 = voiceFm2Osc[0].GetModulatorOutput() * voiceFm2Osc[0].GetIndex();
            }
            if (envelopes[2].envSig > 0.0f) {
                output3 = voiceFm2Osc[2].GetModulatorOutput() * voiceFm2Osc[2].GetIndex();
            }
        }

        // trying out unipolar FM for now ...
        out[2][i] = output2 > 0.0f ? output2 : 0.0f;
        out[3][i] = output3 > 0.0f ? output3 : 0.0f;
    }

    // Update global audio sample counter for high-resolution scheduling
    IncrementAudioSampleCounter(size);
    
    // Note: Display updates moved to main loop to prevent audio dropouts
}

void InitEnvelopes(float samplerate)
{
    adsrLatchEnabled           = ShouldEnableLatch(appState.adsrSustain);
    adsrSustainFullLevelLocked = IsFullLevelZone(appState.adsrSustain);
    currentSustainLevel        = ComputeSustainLevelFromNormalized(appState.adsrSustain);

    for(int i = 0; i < 4; i++)
    {
        // Initialize envelope objects
        envelopes[i].env.Init(samplerate, blocksize);
        
        // Initialize gate state to false (no notes playing initially)
        envelopes[i].gate = false;
        envelopes[i].noteGate = false;
        envelopes[i].latchActive = false;
        envelopes[i].trig = false;
        envelopes[i].env.SetSustainLevel(currentSustainLevel);
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

void SendChannelPressure(uint8_t channel, uint8_t pressure)
{
    // Send MIDI Channel Pressure (Aftertouch) message
    // Format: 0xD0 + channel, pressure
    uint8_t bytes[2] = {static_cast<uint8_t>(0xD0 + channel), pressure};
    hw.midi.SendMessage(bytes, 2);
}

void SendPolyphonicKeyPressure(uint8_t channel, uint8_t note, uint8_t pressure)
{
    // Send MIDI Polyphonic Key Pressure (Aftertouch) message
    // Format: 0xA0 + channel, note, pressure
    uint8_t bytes[3] = {static_cast<uint8_t>(0xA0 + channel), note, pressure};
    hw.midi.SendMessage(bytes, 3);
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
        case ChannelPressure:
        {
            // Forward channel pressure (aftertouch) to channel 15
            ChannelPressureEvent event = m.AsChannelPressure();
            SendChannelPressure(15, event.pressure);
            break;
        }
        case PolyphonicKeyPressure:
        {
            // Forward polyphonic key pressure (aftertouch) to channel 15
            PolyphonicKeyPressureEvent event = m.AsPolyphonicKeyPressure();
            SendPolyphonicKeyPressure(15, event.note, event.pressure);
            break;
        }
        case ControlChange:
        {
            ControlChangeEvent event = m.AsControlChange();
            ProcessHandlerChainControlChange(event);
            break;
        }
        default: break;
    }
}

int main(void)
{
    float samplerate;
    hw.Init();
    
    samplerate = hw.AudioSampleRate();
    SetAudioSampleRate(samplerate);
    ResetAudioSampleCounter();
    
    // Initialize envelopes FIRST (before loading settings)
    // This ensures the envelope objects exist before SetParamValue tries to configure them
    InitEnvelopes(samplerate);

    // Initialize SD Card
    SdmmcHandler::Config sd_cfg;
    sd_cfg.Defaults();
    SdmmcHandler::Result sd_result = sdcard.Init(sd_cfg);
    
    // const char* initMessage = "init failed";
    if (sd_result == SdmmcHandler::Result::OK) {
        uint8_t bsp_result = BSP_SD_Init();
        if (bsp_result == MSD_OK) {
            sdCardInitialized = true;
            // Small delay to ensure card is ready
            // HAL_Delay(100);  // 100ms delay
            // initMessage = LoadPreset() ? "" : "fail";
            // LoadPreset();
        }
    }
    // SetDebugMessage(initMessage);

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
    appState.tuningBpm = bpmKnobValue;
    appState.seqBpm = bpmKnobValue;  // Keep deprecated for backward compatibility

    UpdateOled();

    // start MIDI handler
    hw.midi.StartReceive();

    for (int i = 0; i < 4; i++)
    {
        voices[i].note = 60;
        voices[i].velocity = 0;

        // pluck init
        // plucks[i].decay = 1.0;
        // plucks[i].wetDry = 0.5;
        // plucks[i].synth.SetDecay(1.0);
        // plucks[i].synth.Init(samplerate);

    }
    // synth.Init(samplerate);

    // 
    InitPan(samplerate);
    
    // Initialize limiter
    limiter.Init();
    
    // Initialize delay lines
    delaySampleRate = samplerate;
    delayL.Init();
    delayR.Init();
    delayL.Reset();
    delayR.Reset();
    
    // Clear delay buffers by writing zeros
    for (size_t i = 0; i < MAX_DELAY_SAMPLES; i++) {
        delayL.Write(0.0f);
        delayR.Write(0.0f);
    }
    
    delayFilterStateL = 0.0f;  // Initialize filter states for pluck-like damping
    delayFilterStateR = 0.0f;
    UpdateDelayTime();  // Initialize delay time calculation (also sets read positions)
    
    // Initialize microsound buffers (separate from delay for stability)
    microsoundBufferL.Init();
    microsoundBufferR.Init();
    microsoundBufferL.Reset();
    microsoundBufferR.Reset();
    
    // Clear microsound buffers by writing zeros
    for (size_t i = 0; i < MICROSOUND_BUFFER_SIZE; i++) {
        microsoundBufferL.Write(0.0f);
        microsoundBufferR.Write(0.0f);
    }
    
    // Initialize microsound
    pulsarSynth.Init(samplerate);
    pulsarSynth.SetFrequency(MidiNoteToFrequency(60, 1));  // Default middle C
    pulsarSynth.SetPulsaretLength(10.0f);  // Default 10ms (mid-range)
    pulsarSynth.SetPulseWidth(0.7f);       // Default 70% (audible but not overwhelming)
    pulsarSynth.SetModulation(0.0f);       // Default no modulation
    pulsarSynth.SetWetDry(0.0f);           // Default 0% wet (fully dry)
    
    // Initialize interpolated oscillators for all 4 voices
    for (int i = 0; i < 4; i++) {
        voiceInterpOsc[i].Init(samplerate);
        voiceInterpOsc[i].SetFreq(0.5f);  // Set to 0 so phase doesn't auto-increment
        voiceInterpOsc[i].SetAmp(1.0f);
        voiceInterpOsc[i].SetWaveformParam(0.0f);  // Start with sine wave
    }
    
    // Initialize internal phase oscillators for all 4 voices
    // These follow MIDI note frequencies and are used for phase addressing (voices 1 and 3)
    for (int i = 0; i < 4; i++) {
        internalPhaseOsc[i].Init(samplerate);
        internalPhaseOsc[i].SetFreq(440.0f);  // Start at A4, will be updated by MIDI
        internalPhaseOsc[i].SetAmp(1.0f);
        internalPhaseOsc[i].SetWaveformParam(0.0f);  // Start with sine wave
    }
    
    // Initialize FM2 oscillators
    for (int i = 0; i < 4; i++) {
        voiceFm2Osc[i].Init(samplerate);
        voiceFm2Osc[i].SetFrequency(440.0f);
        voiceFm2Osc[i].SetRatio(1.0f);
        voiceFm2Osc[i].SetIndex(1.0f);
    }
    
    // Initialize Harmonic oscillators
    for (int i = 0; i < 4; i++) {
        voiceHarmonicOsc[i].Init(samplerate);
        voiceHarmonicOsc[i].SetFreq(440.0f);
        voiceHarmonicOsc[i].SetFirstHarmIdx(1);
        float amplitudes[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        voiceHarmonicOsc[i].SetAmplitudes(amplitudes);
    }
    
    // Initialize sine lookup table for fast binaural oscillators (no trig calls in audio loop)
    if (!sineLUTInitialized) {
        for (int i = 0; i < 256; i++) {
            sineLUT[i] = sinf(i * 2.0f * M_PI / 256.0f);
        }
        // Initialize equal power panning lookup tables
        for (int i = 0; i < 256; i++) {
            float pan = (i / 127.5f) - 1.0f;  // -1.0 to +1.0
            float angle = (pan + 1.0f) * M_PI * 0.25f;
            panLeftLUT[i] = cosf(angle);
            panRightLUT[i] = sinf(angle);
        }
        sineLUTInitialized = true;
    }

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
        
        // Note: Sequencer note-off timing is now handled per-track in SequencerMidiSource::Process()

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
                return std::string(FormatMs(appState.adsrAttackMs));
            case 1: // Decay/Release time - convert normalized value to milliseconds
                return std::string(FormatMs(appState.adsrDecayReleaseMs));
            case 2: // Sustain level
                return std::string(FormatPercent(normalizedValue));
            case 3: // Minimum level
                return std::string(FormatPercent(normalizedValue));
            default: return "0%";
        }
    }
    else if (panelId == 'm') {
        switch(paramIndex) {
            case 0: // Frequency
                {
                    static char buf[8];
                    snprintf(buf, sizeof(buf), "%dHz", static_cast<int>(LinearMap(normalizedValue, 0.0f, ParamConfig::PAN_FREQ_MAX_HZ)));
                    return std::string(buf);
                }
            case 1: // Amplitude
                return std::string(FormatPercent(normalizedValue));
            case 2: // Waveform
                if (normalizedValue < 0.25f) return STR_SINE;
                else if (normalizedValue < 0.5f) return STR_TRI;
                else if (normalizedValue < 0.75f) return STR_SQR;
                else return STR_SAW;
            case 3: // Volume
                return std::string(FormatPercent(normalizedValue));
            default: return "0%";
        }
    }
    else if (panelId == 'o') {
        switch(paramIndex) {
            case 0: // Knob 1 - Mode specific
                {
                    int mode = static_cast<int>(appState.oscMode * 2.99f);  // 0-2: Interpolated, FM2, Harmonic
                    mode = std::max(0, std::min(2, mode));
                    switch(mode) {
                        case 0: // Interpolated - waveform
                            if (normalizedValue < 0.25f) return STR_SINE;
                            else if (normalizedValue < 0.5f) return STR_TRI;
                            else if (normalizedValue < 0.75f) return STR_SQR;
                            else return STR_SAW;
                        case 1: // FM2 - ratio
                            {
                                float ratio = 0.125f * powf(64.0f, normalizedValue);
                                ratio = std::max(0.125f, std::min(8.0f, ratio));
                                static char buf[8];
                                snprintf(buf, sizeof(buf), "%d%%", static_cast<int>(ratio * 100.0f));
                                return std::string(buf);
                            }
                        case 2: // Harmonic - idx
                            {
                                int idx = 1 + static_cast<int>(normalizedValue * 15.99f);
                                idx = std::max(1, std::min(16, idx));
                                static char buf[4];
                                snprintf(buf, sizeof(buf), "H%d", idx);
                                return std::string(buf);
                            }
                        default:
                            return STR_SINE;
                    }
                }
            case 1: // Knob 2 - Mode specific
                {
                    int mode = static_cast<int>(appState.oscMode * 2.99f);  // 0-2: Interpolated, FM2, Harmonic
                    mode = std::max(0, std::min(2, mode));
                    switch(mode) {
                        case 0: // Interpolated - empty
                            return "";
                        case 1: // FM2 - index
                            {
                                static char buf[8];
                                snprintf(buf, sizeof(buf), "%d%%", static_cast<int>(normalizedValue * 100));
                                return std::string(buf);
                            }
                        case 2: // Harmonic - decay
                            {
                                float decay = normalizedValue * 10.0f;
                                static char buf[6];
                                snprintf(buf, sizeof(buf), "%dx", static_cast<int>(decay * 10.0f));
                                return std::string(buf);
                            }
                        default:
                            return "";
                    }
                }
            case 2: // Knob 3 - Mode specific
                {
                    int mode = static_cast<int>(appState.oscMode * 2.99f);  // 0-2: Interpolated, FM2, Harmonic
                    mode = std::max(0, std::min(2, mode));
                    switch(mode) {
                        case 0: // Interpolated - empty
                        case 1: // FM2 - empty
                            return "";
                        case 2: // Harmonic - skew
                            return std::to_string(static_cast<int>(normalizedValue * 100)) + "%";
                        default:
                            return "";
                    }
                }
            case 3: // Oscillator Mode
                {
                    int mode = static_cast<int>(normalizedValue * 2.99f); // 0-2: Interpolated, FM2, Harmonic
                    mode = std::max(0, std::min(2, mode));
                    switch(mode) {
                        case 0: return STR_INTER;
                        case 1: return "FM2";
                        case 2: return "Harm";
                        default: return STR_INTER;
                    }
                }
            default: return "Inter";
        }
    }
    else if (panelId == 't') {
        switch(paramIndex) {
            case 0: // Tuning selector
                {
                    static char buf[4];
                    snprintf(buf, sizeof(buf), "%d/9", static_cast<int>(normalizedValue * 9));
                    return std::string(buf);
                }
            case 2: // MIDI
                return normalizedValue > 0.5f ? STR_ON : STR_OFF;
            case 3: // BPM
                {
                    // Map 0-1 to CLOCK_BPM_MIN-CLOCK_BPM_MAX and quantize to multiple of 5
                    int32_t bpm = CLOCK_BPM_MIN + static_cast<int32_t>(normalizedValue * (CLOCK_BPM_MAX - CLOCK_BPM_MIN));
                    bpm = ((bpm + 2) / 5) * 5;  // Round to nearest multiple of 5
                    bpm = std::max(CLOCK_BPM_MIN, std::min(CLOCK_BPM_MAX, bpm));
                    return std::to_string(static_cast<int>(bpm));
                }
            default: return "OFF";
        }
    }
    // Sequencer Track panels (SEQ0, SEQ1, SEQ2, SEQ3) - panel IDs '0', '1', '2', '3'
    else if (panelId == '0' || panelId == '1' || panelId == '2' || panelId == '3') {
        switch(paramIndex) {
            case 0: // Density
                {
                    static char buf[8];
                    snprintf(buf, sizeof(buf), "%d/%d", static_cast<int>(normalizedValue * ParamConfig::SEQ_DENSITY_MAX), ParamConfig::SEQ_DENSITY_MAX);
                    return std::string(buf);
                }
            case 1: // Order
                {
                    // Map normalized value (0.0-1.0) to 6 modes
                    if (normalizedValue < 0.1667f) {
                        return STR_ASC;
                    } else if (normalizedValue < 0.3333f) {
                        return STR_DESC;
                    } else if (normalizedValue < 0.5f) {
                        return STR_UPD;
                    } else if (normalizedValue < 0.6667f) {
                        return STR_FWD;
                    } else if (normalizedValue < 0.8333f) {
                        return STR_RND;
                    } else {
                        return STR_BRN;
                    }
                }
            case 2: // CC Probability
                {
                    uint8_t probability = static_cast<uint8_t>(normalizedValue * 100.0f);
                    probability = std::min(static_cast<uint8_t>(100), probability);
                    static char buf[8];
                    if (probability == 0) return STR_OFF;
                    snprintf(buf, sizeof(buf), "%d%%", probability);
                    return std::string(buf);
                }
            case 3: // Offset
                {
                    // Map normalized value to 7 discrete values in semitones: -12, -5, -4, 0, 4, 5, 12
                    int offsetSemitones;
                    if (normalizedValue < 0.142857f) {
                        offsetSemitones = -12;
                    } else if (normalizedValue < 0.285714f) {
                        offsetSemitones = -5;
                    } else if (normalizedValue < 0.428571f) {
                        offsetSemitones = -4;
                    } else if (normalizedValue < 0.571429f) {
                        offsetSemitones = 0;
                    } else if (normalizedValue < 0.714286f) {
                        offsetSemitones = 4;
                    } else if (normalizedValue < 0.857143f) {
                        offsetSemitones = 5;
                    } else {
                        offsetSemitones = 12;
                    }
                    static char buf[8];
                    snprintf(buf, sizeof(buf), "%d", offsetSemitones);
                    return std::string(buf);
                }
            default: return "0";
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
                    static char buf[8];
                    if (probability == 0) return STR_OFF;
                    snprintf(buf, sizeof(buf), "%d%%", probability);
                    return std::string(buf);
                }
            default: return "OFF";
        }
    }
    else if (panelId == 'd') {
        switch(paramIndex) {
            case 0: // Mode
                return normalizedValue < 0.5f ? STR_SHORT : STR_LONG;
            case 1: // Time
                {
                    // Check if we're in short or long mode
                    float mode = GetParamValue(PARAM_DELAY_MODE);
                    bool isShortMode = mode < 0.5f;
                    
                    if (isShortMode) {
                        // Short mode: use centralized helper
                        float delayMs = DelayTimeToMs(normalizedValue, true);
                        return FormatMs(delayMs);
                    } else {
                        // Long mode: show beat fraction
                        int index = static_cast<int>(normalizedValue * (ParamConfig::NUM_BEAT_MULTIPLIERS - 0.01f));
                        index = std::max(0, std::min(ParamConfig::NUM_BEAT_MULTIPLIERS - 1, index));
                        float beatMult = ParamConfig::BEAT_MULTIPLIERS[index];
                        
                        // Format as fraction or whole number
                        if (index == 0) return "1/4";
                        if (index == 1) return "1/3";
                        if (index == 2) return "1/2";
                        return std::to_string(static_cast<int>(beatMult));
                    }
                }
            case 2: // Damp
                return FormatPercent(normalizedValue);
            case 3: // Wet/Dry
                return FormatPercent(normalizedValue);
            default: return "0%";
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

// Get dynamic OSC panel label names based on current mode - returns const char* to save memory
void GetOscLabels(int mode, const char*& label1, const char*& label2, const char*& label3) {
    static const char* wave = "Wave";
    static const char* ratio = "Ratio";
    static const char* index = "Index";
    static const char* idx = "Idx";
    static const char* decay = "Decay";
    static const char* skew = "Skew";
    static const char* empty = "";
    
    switch(mode) {
        case 0: // Interpolated
            label1 = wave;
            label2 = empty;
            label3 = empty;
            break;
        case 1: // FM2
            label1 = ratio;
            label2 = index;
            label3 = empty;
            break;
        case 2: // Harmonic
            label1 = idx;
            label2 = decay;
            label3 = skew;
            break;
        default:
            label1 = wave;
            label2 = empty;
            label3 = empty;
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
    const char* panelName = currentPanel.name;
    if (panelName) {
        for (size_t i = 0; panelName[i] != '\0' && i < 7; i++) {
            char charStr[2] = {panelName[i], '\0'};
            hw.display.SetCursor(2, startY + i * 9);  // 9 pixels between rows for font_s
            hw.display.WriteString(charStr, font_s, true);
        }
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
        oscMode = static_cast<int>(appState.oscMode * 2.99f);  // 0-2: Interpolated, FM2, Harmonic
        oscMode = std::max(0, std::min(2, oscMode));
        const char* label1, *label2, *label3;
        GetOscLabels(oscMode, label1, label2, label3);
        WriteFixedString(hw, knobPositions[0], labelY, 5, font_s, label1);
        WriteFixedString(hw, knobPositions[1], labelY, 5, font_s, label2);
        WriteFixedString(hw, knobPositions[2], labelY, 5, font_s, label3);
        WriteFixedString(hw, knobPositions[3], labelY, 5, font_s, currentPanel.input4Name);
    } else {
        WriteFixedString(hw, knobPositions[0], labelY, 5, font_s, currentPanel.input1Name);
        WriteFixedString(hw, knobPositions[1], labelY, 5, font_s, currentPanel.input2Name);
        WriteFixedString(hw, knobPositions[2], labelY, 5, font_s, currentPanel.input3Name);
        WriteFixedString(hw, knobPositions[3], labelY, 5, font_s, currentPanel.input4Name);
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
                case 2: // Harmonic
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
                case 2: // Harmonic
                    if (i == 0) paramValue = appState.harmonicIdx;
                    else if (i == 1) paramValue = appState.harmonicDecay;
                    else if (i == 2) paramValue = appState.harmonicSkew;
                    break;
            }
        }
        
        std::string paramValueStr = FormatParameterValue(currentPanel.id, i, paramValue);
        WriteFixedString(hw, knobPositions[i], paramValueY, 5, font_s, paramValueStr.c_str());
    }
    
    // Show sequence pattern for individual SEQ panels (SEQ1, SEQ2, SEQ3, SEQ4)
    if (currentPanel.id == '0' || currentPanel.id == '1' || currentPanel.id == '2' || currentPanel.id == '3') {
        // Map panel ID to track index: '0' -> 0, '1' -> 1, '2' -> 2, '3' -> 3
        int trackIdx = currentPanel.id - '0';
        
        // Show sequence pattern for this track as dots
        char patternStr[TRIGGER_SEQUENCE_LENGTH + 1];
        for (int i = 0; i < TRIGGER_SEQUENCE_LENGTH; i++) {
            patternStr[i] = sequencer.tracks[trackIdx].triggerSequence[i] ? '*' : '-';
        }
        patternStr[TRIGGER_SEQUENCE_LENGTH] = '\0';
        WriteFixedString(hw, knobPositions[0], 32, TRIGGER_SEQUENCE_LENGTH, font_s, patternStr);
        
        // Show sequencer-specific information
        if (sequencer.sequencerMode) {
            // Show step counter with fixed width
            WriteFixedStringF(hw, knobPositions[0], 24, 8, font_s, "Step:%02d", sequencer.currentSequenceStep);
            
            // Show note ordering mode for this track
            const char* modeStr;
            switch (sequencer.tracks[trackIdx].orderMode) {
                case SEQ_ORDER_ASC: modeStr = STR_ASC; break;
                case SEQ_ORDER_DESC: modeStr = STR_DESC; break;
                case SEQ_ORDER_UPD: modeStr = STR_UPD; break;
                case SEQ_ORDER_FWD: modeStr = STR_FWD; break;
                case SEQ_ORDER_RND: modeStr = STR_RND; break;
                case SEQ_ORDER_BRN: modeStr = STR_BRN; break;
                default: modeStr = STR_ASC; break;
            }
            WriteFixedString(hw, knobPositions[3], 32, 4, font_s, modeStr);
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
            const char* display = STR_OFF;
            static char probBuf[8];
            if (probability > 0) {
                snprintf(probBuf, sizeof(probBuf), "%d%%", probability);
                display = probBuf;
            }
            WriteFixedStringF(hw, knobPositions[i], 24, 6, font_s, "%s:%s", slotLabels[i], display);
        }
        
        // Show global note counter and queue status - move to avoid bottom-right area
        // WriteFixedStringF(hw, knobPositions[0], 32, 8, font_s, "Note:%d", globalNoteCounter);
        // WriteFixedStringF(hw, knobPositions[2], 32, 4, font_s, "Q:%d", ccQueueCount);
    }
    // PRESET panel - show first 4 knob normalizedValues
    else if (currentPanel.id == 'p') {
        ShowPresetValues();
    }
    // BNRL (Binaural) panel - show On/Off and Hz value
    else if (currentPanel.id == 'b') {
        // Show On/Off state for enable parameter
        const char* enableStr = binauralEnabled ? "On" : "Off";
        WriteFixedString(hw, knobPositions[0], 24, 4, font_s, enableStr);
        
        // Show spread in Hz (rounded to integer)
        int spreadHz = static_cast<int>(binauralSpreadHz + 0.5f);
        WriteFixedStringF(hw, knobPositions[1], 24, 5, font_s, "%dHz", spreadHz);
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
            // Special handling for OSC panel: knobs 0-2 are used for mode-specific parameters
            // even though they may be PARAM_NONE in the bindings, so don't mark them as caught up
            if (currentPanel.id == 'o' && i < 3) {
                knobCaughtUp[i] = false;  // Force catch-up check for mode-specific params
            } else {
                // If knob is unbound (PARAM_NONE), mark as caught up immediately
                knobCaughtUp[i] = (knobParams[i] == PARAM_NONE);
            }
            // Reset previous knob state to prevent false change detection after panel switch
            previousKnobState[i] = smoothedKnobState[i];
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
                    // SetDebugMessage("SD not init");
                }
                UpdateOled();
            } else {
                // Not in PRESET panel - toggle sequencer mode as normal
                sequencer.sequencerMode = !sequencer.sequencerMode;
                
                if (sequencer.sequencerMode) {
                    // Initialize tracks from UserState (using new per-track parameters)
                    for (int i = 0; i < 4; i++) {
                        float density = GetTrackDensity(i);
                        float orderVal = GetTrackOrder(i);
                        float offsetVal = GetTrackOffset(i);
                        
                        // Set track parameters using helper functions
                        SetTrackDensity(i, density);
                        SetTrackOrder(i, orderVal);
                        SetTrackOffset(i, offsetVal);
                        
                        // Use default note length (50%) - len parameter is deprecated
                        sequencer.tracks[i].noteLengthPercent = 0.5f;
                        sequencer.tracks[i].Reset();
                        // For DESC mode, start at the end of the array
                        if (sequencer.tracks[i].orderMode == SEQ_ORDER_DESC && !sequencer.sequencerNotes.empty()) {
                            sequencer.tracks[i].noteIndex = sequencer.sequencerNotes.size() - 1;
                        }
                    }

                    
                    // Initialize sequence step timing with sample accuracy
                    UpdateSequencerTiming();
                    sequencer.lastSequenceStepTime = hw.seed.system.GetNow();
                    sequencer.currentSequenceStep = 0;
                    uint64_t currentSample = ReadAudioSampleCounter();
                    sequencer.lastSequenceStepSample = currentSample;
                    
                    // When enabling sequencer mode, capture currently held notes
                    CaptureCurrentlyHeldNotes();
                    
                    // Check if step 0 should trigger for any track and play notes immediately
                    if (!sequencer.sequencerNotes.empty()) {
                        for (int trackIdx = 0; trackIdx < 4; trackIdx++) {
                            SequencerTrack& track = sequencer.tracks[trackIdx];
                            if (track.triggerSequence[0]) {
                                // Step 0 triggers for this track - play note immediately
                                // Select note based on order mode (simplified version for initial trigger)
                                uint8_t noteToTrigger = 0;
                                if (!sequencer.sequencerNotes.empty()) {
                                    noteToTrigger = sequencer.sequencerNotes[track.noteIndex % sequencer.sequencerNotes.size()];
                                    // Advance noteIndex for next trigger (matching SelectNoteForTrack behavior)
                                    if (track.orderMode == SEQ_ORDER_ASC || track.orderMode == SEQ_ORDER_FWD) {
                                        track.noteIndex = (track.noteIndex + 1) % sequencer.sequencerNotes.size();
                                    } else if (track.orderMode == SEQ_ORDER_DESC) {
                                        // DESC mode: go backward
                                        if (track.noteIndex == 0) {
                                            track.noteIndex = sequencer.sequencerNotes.size() - 1;
                                        } else {
                                            track.noteIndex--;
                                        }
                                    }
                                }
                                
                                NoteOnEvent event;
                                event.note = noteToTrigger;
                                event.velocity = 127;
                                event.channel = trackIdx;
                                
                                ProcessHandlerChainNoteOn(event);
                                
                                // Schedule note-off
                                if (!shiftRegisterMode) {
                                    uint32_t noteOffDelaySamples = GetNoteOffDelaySamples(track.noteLengthPercent);
                                    track.noteOffSample = currentSample + noteOffDelaySamples;
                                    track.noteOffPending = true;
                                    track.noteToTurnOff = noteToTrigger;
                                    track.voiceToTurnOff = trackIdx;
                                }
                                
                                // Note: SelectNoteForTrack already advances noteIndex for ASC, DESC, FWD modes
                                // For RND and BRN modes, it sets noteIndex directly
                            }
                        }
                    }
                } else {
                    // Clear sequencer notes when disabling sequencer mode to prevent artifacts
                    ClearSequencerNotes();
                    ResetCCState(); // Reset CC state when disabling sequencer
                    
                    // Update CC slot arrays from track values (for when SEQ mode is off)
                    for (int i = 0; i < 4; i++) {
                        SequencerTrack& track = sequencer.tracks[i];
                        int slotBase = i * 2;
                        
                        // Update CC probability from per-track CC probability parameter
                        float ccProb;
                        switch(i) {
                            case 0: ccProb = appState.seq0CcProb; break;
                            case 1: ccProb = appState.seq1CcProb; break;
                            case 2: ccProb = appState.seq2CcProb; break;
                            case 3: ccProb = appState.seq3CcProb; break;
                            default: ccProb = 0.0f; break;
                        }
                        uint8_t prob = static_cast<uint8_t>(ccProb * 100.0f);
                        prob = std::min(static_cast<uint8_t>(100), prob);
                        ccSlotProbabilities[slotBase] = prob;
                        ccSlotProbabilities[slotBase + 1] = prob;
                    }
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
    float knobThreshold = 0.00005; // lower normalizedValues for slower movement
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
        // all other panels are handled by SetParamValue() via bindings
        if (currentPanel.id == 'o' && inputIndex < 3) {
            int mode = static_cast<int>(appState.oscMode * 2.99f);  // 0-2: Interpolated, FM2, Harmonic
            mode = std::max(0, std::min(2, mode));
            
            ParamId modeSpecificParam = PARAM_NONE;
            switch(mode) {
                case 0: // Interpolated
                    if (inputIndex == 0) modeSpecificParam = PARAM_OSC_WAVEFORM;
                    break;
                case 1: // FM2
                    if (inputIndex == 0) modeSpecificParam = PARAM_OSC_FM2_RATIO;
                    else if (inputIndex == 1) modeSpecificParam = PARAM_OSC_FM2_INDEX;
                    break;
                case 2: // Harmonic
                    if (inputIndex == 0) modeSpecificParam = PARAM_OSC_HARMONIC_IDX;
                    else if (inputIndex == 1) modeSpecificParam = PARAM_OSC_HARMONIC_DECAY;
                    else if (inputIndex == 2) modeSpecificParam = PARAM_OSC_HARMONIC_SKEW;
                    break;
            }
            paramId = modeSpecificParam;
        }
        
        // Implement catch-up logic to prevent parameter jumps when switching panels
        if (paramId != PARAM_NONE) {
            float knobPosition = inputs[inputIndex];
            
            // Apply a deadzone to the microsound mix knob so it truly stays at 0% when idle
            if (paramId == PARAM_MICRO_WETDRY && knobPosition < MICROSOUND_MIN_ACTIVE_MIX) {
                knobPosition = 0.0f;
            }
            
            // For OSC panel mode-specific parameters, compare against actual parameter value
            // instead of stored knob position, since the same physical knob controls different
            // parameters depending on the mode
            float targetValue;
            bool isOscModeSpecificParam = (paramId == PARAM_OSC_FM2_RATIO || 
                                          paramId == PARAM_OSC_FM2_INDEX ||
                                          paramId == PARAM_OSC_HARMONIC_IDX ||
                                          paramId == PARAM_OSC_HARMONIC_DECAY ||
                                          paramId == PARAM_OSC_HARMONIC_SKEW);
            
            if (isOscModeSpecificParam) {
                // Use actual parameter value for catch-up comparison
                targetValue = GetParamValue(paramId);
            } else {
                // Use stored knob position for catch-up comparison (standard behavior)
                targetValue = knobValues[panelMode][inputIndex];
            }
            
            // Check if knob has caught up to the target normalizedValue
            if (!knobCaughtUp[inputIndex]) {
                // Check if knob is within threshold of target normalizedValue
                if (fabs(knobPosition - targetValue) < KNOB_CATCHUP_THRESHOLD) {
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
            SetParamValue(paramId, knobPosition);
            
            // Update knob normalizedValues storage
            knobValues[panelMode][inputIndex] = knobPosition;
            
            knobChanged = true;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        previousKnobState[i] = smoothedKnobState[i]; // Update with smoothed normalizedValues for next comparison
    }
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
    
    // Initialize pan LFO oscillator
    panLfo.Init(samplerate);
    panLfo.SetFreq(panFreq);
    panLfo.SetAmp(1.0f);
    panLfo.SetWaveformParam(0.0f);  // Default sine wave
    
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
            // Always update oscillator frequencies for all voices
            float freq = MidiNoteToFrequency(static_cast<int8_t>(state.note), 1);  // Use 1 to avoid voice 0 special case
            SetOscillatorFrequency(i, freq);
            
            // Send pitch bend for shift register mode if tuning is enabled
            if (sendPitchBendMidi && state.gate_on) {
                const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                float centsDeviation = CalculateCentsDeviation(static_cast<uint8_t>(state.note), tuning);
                
                // Add binaural detuning for voices 1 and 3 (analog oscillator inputs via MIDI)
                if (binauralEnabled && (i == 1 || i == 3)) {
                    float baseFreq = 440.0f * powf(2.0f, (state.note - 69) / 12.0f);
                    centsDeviation += HzDetuningToCents(baseFreq, binauralSpreadHz / 2.0f);
                }
                
                int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                SendPitchBend(static_cast<uint8_t>(i), pitchBendValue);
            }
            
            if(state.needs_retrigger)
            {
                // Set sustain level before retriggering
                float sustainLevel = currentSustainLevel;
                if(adsrSustainFullLevelLocked)
                {
                    sustainLevel = 1.0f;
                }
                else
                {
                    float velocityFactor = state.velocity / 127.0f;
                    sustainLevel *= velocityFactor;
                }
                envelopes[i].env.SetSustainLevel(sustainLevel);
                
                envelopes[i].env.Retrigger(true);
            }
            // Use gate_on state from the library (tracks note-on/off)
            envelopes[i].noteGate   = state.gate_on;
            envelopes[i].latchActive = adsrLatchEnabled && state.gate_on;
            envelopes[i].gate  = state.gate_on || envelopes[i].latchActive;
            voices[i].note     = static_cast<int8_t>(state.note);
            voices[i].velocity = static_cast<int8_t>(state.velocity);
        }
        else
        {
            envelopes[i].gate      = false;
            envelopes[i].noteGate  = false;
            envelopes[i].latchActive = false;
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
    
    bool wasEmpty = sequencer.sequencerNotes.empty();
    
    // Add note to array
    sequencer.sequencerNotes.push_back(note);
    
    // Check if any track needs sorting (i.e., is not in FWD mode)
    // If all tracks are FWD, preserve order as notes were added
    bool needsSort = false;
    for (int i = 0; i < 4; i++) {
        if (sequencer.tracks[i].orderMode != SEQ_ORDER_FWD) {
            needsSort = true;
            break;
        }
    }
    if (needsSort) {
        SortSequencerNotes();
    }
    
    // Only reset note indices if this was the first note (sequencer was empty)
    // Otherwise, preserve current position to avoid disrupting playback
    if (wasEmpty) {
        for (int i = 0; i < 4; i++) {
            sequencer.tracks[i].noteIndex = 0;
            // For DESC mode, start at the end
            if (sequencer.tracks[i].orderMode == SEQ_ORDER_DESC && sequencer.sequencerNotes.size() > 0) {
                sequencer.tracks[i].noteIndex = sequencer.sequencerNotes.size() - 1;
            }
        }
    } else if (needsSort) {
        // If we sorted, we need to find the current note in the new sorted array
        // For now, just reset to 0 (or end for DESC) - this is a limitation but better than breaking playback
        for (int i = 0; i < 4; i++) {
            if (sequencer.tracks[i].orderMode == SEQ_ORDER_DESC && sequencer.sequencerNotes.size() > 0) {
                sequencer.tracks[i].noteIndex = sequencer.sequencerNotes.size() - 1;
            } else {
                sequencer.tracks[i].noteIndex = 0;
            }
        }
    }
    // If FWD mode and not empty, don't reset - preserve current position
    
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
    
    // For brownian mode, reset last note if it was the one removed (for all tracks)
    for (int i = 0; i < 4; i++) {
        if (sequencer.tracks[i].orderMode == SEQ_ORDER_BRN && sequencer.tracks[i].lastNote == note) {
            sequencer.tracks[i].lastNote = 0;  // Reset to trigger random selection next time
        }
        
        // Adjust track note index if needed (out of bounds after note removal)
        if (!sequencer.sequencerNotes.empty() && sequencer.tracks[i].noteIndex >= sequencer.sequencerNotes.size()) {
            // Set to end for DESC mode, 0 for other modes
            if (sequencer.tracks[i].orderMode == SEQ_ORDER_DESC) {
                sequencer.tracks[i].noteIndex = sequencer.sequencerNotes.size() - 1;
            } else {
                sequencer.tracks[i].noteIndex = 0;
            }
        }
    }
}

void SortSequencerNotes()
{
    // Check if any track needs sorting (i.e., is not in FWD mode)
    bool needsSort = false;
    for (int i = 0; i < 4; i++) {
        if (sequencer.tracks[i].orderMode != SEQ_ORDER_FWD) {
            needsSort = true;
            break;
        }
    }
    
    // If all tracks are in FWD mode, don't sort - preserve order as notes were added
    if (!needsSort) {
        return;
    }
    
    // Sort in ascending order - this works for all modes:
    // - ASC: traverse forward (0, 1, 2, ...)
    // - DESC: traverse backward (n-1, n-2, ..., 0)
    // - UPD: traverse up then down (0, 1, 2, ..., n-1, n-2, ..., 0)
    // - RND/BRN: use sorted array for easier calculations
    std::sort(sequencer.sequencerNotes.begin(), sequencer.sequencerNotes.end());
    
    // Reset upDownDirection for UPD mode tracks
    for (int i = 0; i < 4; i++) {
        if (sequencer.tracks[i].orderMode == SEQ_ORDER_UPD) {
            sequencer.tracks[i].upDownDirection = true;
        }
    }
    
    // Reset all tracks' note indices based on their order mode
    for (int i = 0; i < 4; i++) {
        if (sequencer.tracks[i].orderMode == SEQ_ORDER_DESC && !sequencer.sequencerNotes.empty()) {
            // DESC mode: start at the end (highest note)
            sequencer.tracks[i].noteIndex = sequencer.sequencerNotes.size() - 1;
        } else {
            // ASC, FWD, UPD, RND, BRN modes: start at the beginning
            sequencer.tracks[i].noteIndex = 0;
        }
    }
}

// Helper function to update track order mode (implemented after SortSequencerNotes declaration)
inline void SetTrackOrder(int trackIdx, float normalizedValue) {
    SequencerTrack& track = sequencer.tracks[trackIdx];
    SequencerOrderMode newMode = NormalizedToOrderMode(normalizedValue);
    
    if (newMode != track.orderMode) {
        track.orderMode = newMode;
        // Set noteIndex based on order mode for this track only
        if (newMode == SEQ_ORDER_DESC && !sequencer.sequencerNotes.empty()) {
            track.noteIndex = sequencer.sequencerNotes.size() - 1;
        } else {
            track.noteIndex = 0;
        }
        track.upDownDirection = true;
        // Don't call SortSequencerNotes() here - it would affect all tracks
        // The array is already sorted (or unsorted for FWD mode) from when notes were added
        // Each track traverses the array according to its own order mode
    }
}

void ClearSequencerNotes()
{
    sequencer.sequencerNotes.clear();
    for (int i = 0; i < 4; i++) {
        sequencer.tracks[i].noteIndex = 0;
        sequencer.tracks[i].upDownDirection = true;
        sequencer.tracks[i].lastNote = 0;
    }
    sequencer.sequencerUsingInitialCapture = false;  // Reset flag when clearing notes
}

void CaptureCurrentlyHeldNotes()
{
    // Clear existing sequencer notes first
    sequencer.sequencerNotes.clear();
    
    // Capture all currently held notes from the voices array
    for (int i = 0; i < 4; i++) {
        if (envelopes[i].noteGate && voices[i].note > 0) {
            // Add note to sequencer if it's currently held
            AddNoteToSequencer(static_cast<uint8_t>(voices[i].note));
        }
    }
    
    // Reset all tracks' note indices based on their order mode
    for (int i = 0; i < 4; i++) {
        if (sequencer.tracks[i].orderMode == SEQ_ORDER_DESC && !sequencer.sequencerNotes.empty()) {
            sequencer.tracks[i].noteIndex = sequencer.sequencerNotes.size() - 1;
        } else {
            sequencer.tracks[i].noteIndex = 0;
        }
    }
    
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
        if (envelopes[i].noteGate && voices[i].note > 0) {
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
                
                // Add binaural detuning for voices 1 and 3 (analog oscillator inputs via MIDI)
                if (binauralEnabled && (i == 1 || i == 3)) {
                    float baseFreq = 440.0f * powf(2.0f, (voices[i].note - 69) / 12.0f);
                    centsDeviation += HzDetuningToCents(baseFreq, binauralSpreadHz / 2.0f);
                }
                
                int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                SendPitchBend(static_cast<uint8_t>(i), pitchBendValue);
            }
            
            // Always update oscillator frequencies for all voices
            float freq = MidiNoteToFrequency(voices[i].note, 1);  // Use 1 to avoid voice 0 special case
            SetOscillatorFrequency(i, freq);
        }
    }
}

// Trigger Sequence Generator Implementation
void InitTriggerSequence()
{
    // Initialize per-track trigger sequences based on each track's density
    // Each track now has its own independent trigger sequence
    for (int i = 0; i < 4; i++) {
        int numTriggers = static_cast<int>(sequencer.tracks[i].density + 0.5f);
        numTriggers = std::max(0, std::min(numTriggers, static_cast<int>(TRIGGER_SEQUENCE_LENGTH)));
        GenerateEuclideanRhythm(numTriggers, TRIGGER_SEQUENCE_LENGTH, sequencer.tracks[i].triggerSequence);
    }

    // Initialize timing
    sequencer.currentSequenceStep = 0;
    sequencer.lastSequenceStepTime = hw.seed.system.GetNow();
    UpdateSequencerTiming();
    uint64_t currentSample = ReadAudioSampleCounter();
    sequencer.lastSequenceStepSample = currentSample;
    
}


// Add CC to queue - simplified, no timing calculation needed
// Voice 4 (track 3) CCs are prioritized by inserting at head of queue
void AddCCToQueue(uint8_t ccValue, bool isReset) {
    // Check if queue is full
    if (ccQueueCount >= CC_QUEUE_SIZE) {
        // SetDebugMessage("CC Queue Full!");
        return;
    }
    
    // Voice 4 (track 3) uses CC slots 6-7, which map to values 109 and 127
    // Prioritize these by inserting at head instead of tail
    bool isVoice4 = (ccValue == 109 || ccValue == 127);
    
    if (isVoice4) {
        // Insert at head (priority queue - will be processed next)
        ccQueueHead = (ccQueueHead - 1 + CC_QUEUE_SIZE) % CC_QUEUE_SIZE;
        ccQueue[ccQueueHead].ccValue = ccValue;
        ccQueue[ccQueueHead].isReset = isReset;
    } else {
        // Add to tail (normal FIFO behavior)
        ccQueue[ccQueueTail].ccValue = ccValue;
        ccQueue[ccQueueTail].isReset = isReset;
        ccQueueTail = (ccQueueTail + 1) % CC_QUEUE_SIZE;
    }
    
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
        ccLatchTime = currentTime; // Start timing from now
        // SetDebugMessage("CC Init");
    }
    
    // Process next item in queue if enough time has passed since last processing
    // Optimize: process first item immediately if queue was empty, otherwise maintain spacing
    if (ccQueueCount > 0) {
        uint32_t timeSinceLastProcess = currentTime - ccLatchTime;
        
        // Process immediately if:
        // 1. Enough time has passed (normal case - maintains spacing), OR
        // 2. No CCs have been sent yet (first item - reduces initial latency to ~0ms)
        bool canProcess = (timeSinceLastProcess >= CC_RESET_DELAY_MS) || !ccIsLatched;
        
        if (canProcess) {
            CCQueueItem& item = ccQueue[ccQueueHead];
            
            SendMidiMesssage(item.ccValue, 15, "CC");
            lastCCValue = item.ccValue;
            ccLatchTime = currentTime; // Update timing for next item
            ccIsLatched = true;

            SendMidiMesssage(sequencer.triggerNote, sequencer.ccTriggerChannel, "TRIGGER_OFF");
            SendMidiMesssage(sequencer.triggerNote, sequencer.ccTriggerChannel, "TRIGGER_ON");
            ccTriggerOffTime = currentTime + CC_RESET_DELAY_MS;
            ccTriggerOffPending = true;
            
            ccQueueHead = (ccQueueHead + 1) % CC_QUEUE_SIZE;
            ccQueueCount--;
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

// Force immediate reset to clean state
void ResetCCState() {
    ccQueueCount = 0;
    ccQueueHead = 0;
    ccQueueTail = 0;
    SendMidiMesssage(0, sequencer.ccValueChannel, "CC");
    lastCCValue = 0;
    ccIsLatched = false;
    ccStateInitialized = true;
}

bool ShouldFireWithProbability(uint8_t probability) {
    if (probability >= 100) return true;
    if (probability == 0) return false;
    uint8_t randomValue = rand() % 100; // 0-99
    return randomValue < probability;
}

// Process CC slots - called once per sequence step
void ProcessCCSlots()
{
    globalNoteCounter++; // Keep for display purposes
    
    int triggeredSlots = 0;
    uint8_t triggeredValues[8];
    
    if (sequencer.sequencerMode) {
        // SEQ mode ON: Process per-track CC pairs using track trigger sequence and CC probability
        // Track 0 -> CC slots 0-1, Track 1 -> CC slots 2-3, etc.
        for (int trackIdx = 0; trackIdx < 4; trackIdx++) {
            SequencerTrack& track = sequencer.tracks[trackIdx];
            int slotBase = trackIdx * 2;  // 0, 2, 4, 6
            
            // Check if current step triggers for this track
            bool stepTriggers = track.triggerSequence[sequencer.currentSequenceStep];
            
            // Get CC probability and inverse mode for this track from appState
            float ccProb = 0.0f;
            bool ccInverse = false;
            switch(trackIdx) {
                case 0: 
                    ccProb = appState.seq0CcProb; 
                    ccInverse = appState.seq0CcInverse;
                    break;
                case 1: 
                    ccProb = appState.seq1CcProb; 
                    ccInverse = appState.seq1CcInverse;
                    break;
                case 2: 
                    ccProb = appState.seq2CcProb; 
                    ccInverse = appState.seq2CcInverse;
                    break;
                case 3: 
                    ccProb = appState.seq3CcProb; 
                    ccInverse = appState.seq3CcInverse;
                    break;
            }
            
            // Convert CC probability (0.0-1.0) to percentage (0-100)
            uint8_t prob = static_cast<uint8_t>(ccProb * 100.0f);
            prob = std::min(static_cast<uint8_t>(100), prob);  // Clamp to 100
            
            // Determine which slots should fire based on inverse mode
            bool firstSlotShouldFire = false;
            bool secondSlotShouldFire = false;
            
            if (ccInverse) {
                // Inverse mode: first slot follows sequencer, second inverts it
                firstSlotShouldFire = stepTriggers;
                secondSlotShouldFire = !stepTriggers;
            } else {
                // Normal mode: both slots fire when step triggers
                firstSlotShouldFire = stepTriggers;
                secondSlotShouldFire = stepTriggers;
            }
            
            // Check probability for this track's CC pair (same probability applies to both)
            if (ShouldFireWithProbability(prob)) {
                // Fire first CC slot if it should fire
                if (firstSlotShouldFire) {
                    triggeredValues[triggeredSlots++] = ccSlotValues[slotBase];
                }
                // Fire second CC slot if it should fire
                if (secondSlotShouldFire) {
                    triggeredValues[triggeredSlots++] = ccSlotValues[slotBase + 1];
                }
            }
        }
    } else {
        // SEQ mode OFF: Process all CC slots
        for (int i = 7; i >= 0; i--) {
            // Evaluate probability for each slot
            if (ShouldFireWithProbability(ccSlotProbabilities[i])) {
                triggeredValues[triggeredSlots] = ccSlotValues[i];
                triggeredSlots++;
            }
        }
    }
    
    // Queue all triggered CCs (no timing calculation needed - handled by ProcessCCQueue)
    for (int i = 0; i < triggeredSlots; i++) {
        AddCCToQueue(triggeredValues[i], false);
    }
    
    // If no CCs were triggered and we have a high CC normalizedValue, force latch down
    if (triggeredSlots == 0 && lastCCValue > 0) {
        ResetCCState();
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
    bool HandleControlChange(ControlChangeEvent& event) { return false; }
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
    
    bool HandleControlChange(ControlChangeEvent& event) {
        // Always pass to next handler
        return this->GetNext().HandleControlChange(event);
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
                
                // Add binaural detuning for voices 1 and 3 (analog oscillator inputs via MIDI)
                if (binauralEnabled && (event.channel == 1 || event.channel == 3)) {
                    float baseFreq = 440.0f * powf(2.0f, (event.note - 69) / 12.0f);
                    centsDeviation += HzDetuningToCents(baseFreq, binauralSpreadHz / 2.0f);
                }
                
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
    
    bool HandleControlChange(ControlChangeEvent& event) {
        // Always pass to next handler
        return this->GetNext().HandleControlChange(event);
    }
};

// NormalVoiceHandler - direct voice allocation (round-robin)
// If channel is already set (0-3), use it directly (for sequencer notes)
template<typename NextHandler>
class NormalVoiceHandler : public HandlerBase<NextHandler> {
public:
    bool HandleNoteOn(NoteOnEvent& event) {
        // Apply offset for sequencer-generated notes (channel 0-3 in sequencer mode)
        // This is the last possible moment before the note is used
        if (sequencer.sequencerMode && event.channel >= 0 && event.channel < 4) {
            int trackIdx = event.channel;
            SequencerTrack& track = sequencer.tracks[trackIdx];
            int offsetNote = static_cast<int>(event.note) + static_cast<int>(track.offset);
            event.note = static_cast<uint8_t>(std::max(0, std::min(127, offsetNote)));
        }
        
        int8_t voiceIndex = -1;
        
        // Check if channel is already set (0-3) AND we're in sequencer mode - this indicates sequencer routing
        // Regular MIDI input on channels 0-3 should still use round-robin allocation
        if (sequencer.sequencerMode && event.channel >= 0 && event.channel < 4) {
            // Use the pre-set channel (sequencer track routing)
            voiceIndex = event.channel;
            
            // If voice is busy, send note-off for the stolen voice
            if (envelopes[voiceIndex].noteGate && voices[voiceIndex].note > 0) {
                uint8_t noteOffBytes[3] = {
                    static_cast<uint8_t>(0x80 + voiceIndex), 
                    static_cast<uint8_t>(voices[voiceIndex].note), 
                    0
                };
                hw.midi.SendMessage(noteOffBytes, 3);
            }
        } else {
            // Voice allocation: Round-robin distribution across voices 0-3 (for manual keyboard input)
        // Step 1: Try the next voice in round-robin sequence if it's free
        if (!envelopes[nextVoiceIndex].noteGate) {
            voiceIndex = nextVoiceIndex;
        } else {
            // Step 2: If next voice is busy, search for any free voice
            bool foundFree = false;
            for (int i = 0; i < 4; i++) {
                if (!envelopes[i].noteGate) {
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
        
            // Advance round-robin index for next note (only for manual input)
        nextVoiceIndex = (voiceIndex + 1) % 4;
        }
        
        // Send pitch bend before note-on if tuning is enabled
        if (sendPitchBendMidi) {
            const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
            float centsDeviation = CalculateCentsDeviation(event.note, tuning);
            
            // Add binaural detuning for voices 1 and 3 (analog oscillator inputs via MIDI)
            if (binauralEnabled && (voiceIndex == 1 || voiceIndex == 3)) {
                float baseFreq = 440.0f * powf(2.0f, (event.note - 69) / 12.0f);
                centsDeviation += HzDetuningToCents(baseFreq, binauralSpreadHz / 2.0f);
            }
            
            int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
            SendPitchBend(voiceIndex, pitchBendValue);
        }
        
        // Send MIDI note-on to external devices on the allocated voice channel
        uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + voiceIndex), event.note, event.velocity};
        hw.midi.SendMessage(bytes, 3);
        
        // Always update oscillator frequencies for all voices
        float freq = MidiNoteToFrequency(event.note, 1);  // Use 1 to avoid voice 0 special case
        SetOscillatorFrequency(voiceIndex, freq);
        
        // Update voice state
        envelopes[voiceIndex].noteGate = true;
        envelopes[voiceIndex].latchActive = adsrLatchEnabled;
        envelopes[voiceIndex].gate = true;
        voices[voiceIndex].note = event.note;
        voices[voiceIndex].velocity = event.velocity;
        
        // Set velocity-scaled sustain level before retriggering
        float sustainLevel = currentSustainLevel;
        
        // Normal sustain calculation
        if(adsrSustainFullLevelLocked)
        {
            sustainLevel = 1.0f;
        }
        else
        {
            float velocityFactor = event.velocity / 127.0f;
            sustainLevel *= velocityFactor;
        }
        
        envelopes[voiceIndex].env.SetSustainLevel(sustainLevel);
        
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
                envelopes[i].noteGate = false;
                envelopes[i].latchActive = false;
                
                // Clear voice data when note is released
                voices[i].note = 0;
                voices[i].velocity = 0;
            }
        }
        
        // Pass to next handler
        return this->GetNext().HandleNoteOff(event);
    }
    
    bool HandleControlChange(ControlChangeEvent& event) {
        // Always pass to next handler
        return this->GetNext().HandleControlChange(event);
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
        
        // Process CC slots (once per sequence step)
        if (sequencer.sequencerMode) {
            // already handled in sequencer
            // ProcessCCSlots();
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
    
    bool HandleControlChange(ControlChangeEvent& event) {
        // Always pass to next handler
        return this->GetNext().HandleControlChange(event);
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
    
    bool HandleControlChange(ControlChangeEvent& event) {
        // Always pass to next handler
        return this->GetNext().HandleControlChange(event);
    }
};

// ControlHandler - processes MIDI CC messages for voice amplitude control
// CC values 100-103 control amplitude of voices 0-3 respectively
template<typename NextHandler>
class ControlHandler : public HandlerBase<NextHandler> {
public:
    bool HandleNoteOn(NoteOnEvent& event) {
        // Pass through to next handler
        return this->GetNext().HandleNoteOn(event);
    }
    
    bool HandleNoteOff(NoteOffEvent& event) {
        // Pass through to next handler
        return this->GetNext().HandleNoteOff(event);
    }
    
    bool HandleControlChange(ControlChangeEvent& event) {
        // Process CC values 100 and above
        if (event.control_number >= 100) {
            // CC 100-103 control amplitude of voices 0-3
            if (event.control_number >= 100 && event.control_number <= 103) {
                int voiceIndex = event.control_number - 100;
                // Convert CC value (0-127) to amplitude (0.0-1.0)
                float amplitude = event.value / 127.0f;
                appState.voiceAmplitudes[voiceIndex] = amplitude;
            }
            // Handled - stop chain (don't pass to next handler)
            return true;
        }
        // Not handled - pass to next handler
        return this->GetNext().HandleControlChange(event);
    }
};

// Define the handler chain type - compile-time composition
using HandlerChain = SequencerCaptureHandler<
    ShiftRegisterHandler<
        IntellijelPitchHandler<
            NormalVoiceHandler<
                IntellijelTrackerHandler<
                    ControlHandler<NullHandler>
                >
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

void ProcessHandlerChainControlChange(ControlChangeEvent& event) {
    handlerChain.HandleControlChange(event);
}

// SequencerMidiSource - generates notes from sequencer array based on clock
// Now supports four independent tracks, each with its own note count and timing
class SequencerMidiSource {
public:
    // Helper to select next note for a track based on its order mode
    uint8_t SelectNoteForTrack(int trackIdx) {
        SequencerTrack& track = sequencer.tracks[trackIdx];
        
        if (sequencer.sequencerNotes.empty()) {
            return 0;
        }
        
        uint8_t noteToTrigger = 0;
            
        switch (track.orderMode) {
                case SEQ_ORDER_ASC:
                    // Ascending: forward through sorted array
                noteToTrigger = sequencer.sequencerNotes[track.noteIndex];
                track.noteIndex = (track.noteIndex + 1) % sequencer.sequencerNotes.size();
                    break;
                    
                case SEQ_ORDER_DESC:
                    // Descending: backward through sorted array
                noteToTrigger = sequencer.sequencerNotes[track.noteIndex];
                if (track.noteIndex == 0) {
                    track.noteIndex = sequencer.sequencerNotes.size() - 1;
                } else {
                    track.noteIndex--;
                }
                    break;
                    
                case SEQ_ORDER_FWD:
                    // Use sequencerNotes directly - in FWD mode it's not sorted, so it preserves original order
                noteToTrigger = sequencer.sequencerNotes[track.noteIndex];
                track.noteIndex = (track.noteIndex + 1) % sequencer.sequencerNotes.size();
                    break;
                    
                case SEQ_ORDER_UPD:
                    // Up-down: go up then down, reversing at ends
                noteToTrigger = sequencer.sequencerNotes[track.noteIndex];
                if (track.upDownDirection) {
                        // Going up
                    if (track.noteIndex >= sequencer.sequencerNotes.size() - 1) {
                            // Reached end, reverse direction
                        track.upDownDirection = false;
                            if (sequencer.sequencerNotes.size() > 1) {
                            track.noteIndex--;
                            }
                        } else {
                        track.noteIndex++;
                        }
                    } else {
                        // Going down
                    if (track.noteIndex == 0) {
                            // Reached start, reverse direction
                        track.upDownDirection = true;
                            if (sequencer.sequencerNotes.size() > 1) {
                            track.noteIndex++;
                            }
                        } else {
                        track.noteIndex--;
                        }
                    }
                    break;
                    
                case SEQ_ORDER_RND:
                    // Random selection
                track.noteIndex = rand() % sequencer.sequencerNotes.size();
                noteToTrigger = sequencer.sequencerNotes[track.noteIndex];
                    break;
                    
                case SEQ_ORDER_BRN:
                    // Brownian: random distance from last note
                if (track.lastNote == 0 || sequencer.sequencerNotes.size() == 1) {
                        // First note or only one note - pick randomly
                    track.noteIndex = rand() % sequencer.sequencerNotes.size();
                    noteToTrigger = sequencer.sequencerNotes[track.noteIndex];
                    } else {
                        // Find current note index in sorted array
                        size_t currentIdx = 0;
                        for (size_t i = 0; i < sequencer.sequencerNotes.size(); i++) {
                        if (sequencer.sequencerNotes[i] == track.lastNote) {
                                currentIdx = i;
                                break;
                            }
                        }
                        
                        // Random step: -2 to +2 (brownian motion)
                        int step = (rand() % 5) - 2; // -2, -1, 0, 1, 2
                        int newIdx = static_cast<int>(currentIdx) + step;
                        
                        // Clamp to valid range
                        if (newIdx < 0) newIdx = 0;
                        if (newIdx >= static_cast<int>(sequencer.sequencerNotes.size())) {
                            newIdx = static_cast<int>(sequencer.sequencerNotes.size()) - 1;
                        }
                        
                    track.noteIndex = static_cast<size_t>(newIdx);
                    noteToTrigger = sequencer.sequencerNotes[track.noteIndex];
                    }
                track.lastNote = noteToTrigger;
                    break;
            }
        
        // Return the original note - offset will be applied later in the handler chain
        return noteToTrigger;
    }
    
public:
    void Process() {
        if (!sequencer.sequencerMode || sequencer.sequencerNotes.empty()) {
            return;
        }
        
        uint64_t currentSample = ReadAudioSampleCounter();
        
        // Advance sequence steps based on sample-accurate timing
        // Calculate steps elapsed to prevent timing drift when processing multiple steps
        if (sequencer.sequenceStepIntervalSamples > 0) {
            uint64_t samplesSinceLastStep = currentSample - sequencer.lastSequenceStepSample;
            uint64_t stepsElapsed = samplesSinceLastStep / sequencer.sequenceStepIntervalSamples;
            
            // Process each step that should have occurred
            // Limit to TRIGGER_SEQUENCE_LENGTH to prevent excessive processing
            uint64_t stepsToProcess = (stepsElapsed > TRIGGER_SEQUENCE_LENGTH) ? TRIGGER_SEQUENCE_LENGTH : stepsElapsed;
            
            if (stepsToProcess > 0) {
                // Calculate the exact sample position for the first step to process
                // This prevents timing drift by using absolute timing, not accumulation
                uint64_t firstStepSample = sequencer.lastSequenceStepSample + sequencer.sequenceStepIntervalSamples;
                
                for (uint64_t step = 0; step < stepsToProcess; step++) {
                    uint8_t previousStep = sequencer.currentSequenceStep;
                    
                    // Calculate exact sample position for this step (absolute, not accumulated)
                    // This ensures timing is independent of processing time
                    uint64_t stepSample = firstStepSample + (step * sequencer.sequenceStepIntervalSamples);
                    sequencer.lastSequenceStepSample = stepSample;
                    sequencer.currentSequenceStep = (sequencer.currentSequenceStep + 1) % TRIGGER_SEQUENCE_LENGTH;
                    
                    // Removed GetNow() call from inside loop - not needed for sample-accurate timing
                    // This eliminates variable execution time that causes jitter
                    
                    for (int trackIdx = 0; trackIdx < 4; trackIdx++) {
                        SequencerTrack& track = sequencer.tracks[trackIdx];
                        
                        if (sequencer.sequencerNotes.empty() || track.density <= 0.0f) {
                            continue;
                        }
                        
                        bool currentStepTriggers = track.triggerSequence[sequencer.currentSequenceStep];
                        
                        if (currentStepTriggers) {
                            uint8_t noteToTrigger = SelectNoteForTrack(trackIdx);
                            
                            NoteOnEvent event;
                            event.note = noteToTrigger;
                            event.velocity = 127;
                            event.channel = trackIdx;
                            
                            ProcessHandlerChainNoteOn(event);
                            
                            if (!shiftRegisterMode) {
                                uint32_t noteOffDelaySamples = GetNoteOffDelaySamples(track.noteLengthPercent);
                                track.noteOffSample = stepSample + noteOffDelaySamples;
                                track.noteOffPending = true;
                                track.noteToTurnOff = noteToTrigger;
                                track.voiceToTurnOff = trackIdx;
                            }
                        }
                    }
                    
                    // Process CC slots once per sequence step
                    ProcessCCSlots();
                    
                    if (sequencer.currentSequenceStep == 0 && previousStep == TRIGGER_SEQUENCE_LENGTH - 1) {
                        for (int i = 0; i < 4; i++) {
                            sequencer.tracks[i].upDownDirection = true;
                        }
                    }
                }
                
                // Update lastSequenceStepTime once after processing all steps (not in the loop)
                // This reduces system calls and eliminates timing jitter from variable GetNow() execution time
                sequencer.lastSequenceStepTime = hw.seed.system.GetNow();
            }
        }
        
        // Process note-offs for all tracks using sample timestamps
        for (int trackIdx = 0; trackIdx < 4; trackIdx++) {
            SequencerTrack& track = sequencer.tracks[trackIdx];
            if (track.noteOffPending && currentSample >= track.noteOffSample) {
                if (voices[trackIdx].note == track.noteToTurnOff) {
                    uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + trackIdx), track.noteToTurnOff, 0};
                    hw.midi.SendMessage(bytes, 3);
                    
                    envelopes[trackIdx].gate = false;
                    
                    // For latch behavior: sequencer note-offs are just for timing control
                    // Don't clear noteGate or latchActive if latch is enabled
                    // This allows latch to continue working even when sequencer sends note-offs
                    if (!adsrLatchEnabled) {
                        envelopes[trackIdx].noteGate = false;
                        envelopes[trackIdx].latchActive = false;
                    }
                    // If latch is enabled, keep noteGate and latchActive true
                    // so the latch behavior in AudioCallback can continue working
                    
                    voices[trackIdx].note = 0;
                    voices[trackIdx].velocity = 0;
                }
                
                track.noteOffPending = false;
            }
        }
    }
    
    // Public method to advance sequencer step (kept for compatibility)
    void AdvanceSequenceStep() {
        // This is now handled in Process() - kept for compatibility
    }
};

// Global sequencer source instance
SequencerMidiSource sequencerMidiSource;

// Wrapper function for sequencer source
void ProcessSequencerMidiSource() {
    sequencerMidiSource.Process();
}

// Wrapper function to advance sequencer step (kept for compatibility)
void AdvanceSequencerStep() {
    sequencerMidiSource.AdvanceSequenceStep();
}

// Common preset storage constants
static const int PRESET_SECTOR = 2000;
static const int PRESET_BUFFER_LENGTH = 64;  // MAX_PANELS * 4 = 16 * 4
static const int PRESET_TIMEOUT_MS = 5000;
static const int PRESET_MAX_RETRIES = 3;

// Shared preset buffer (saves stack space by reusing between Save/Load)
static uint32_t presetBuffer[PRESET_BUFFER_LENGTH];

bool SavePreset() {
    memset(presetBuffer, 0, sizeof(presetBuffer));
    
    // Store all knob normalizedValues (panelModesCount panels × 4 knobs)
    int idx = 0;
    for (int panel = 0; panel < panelModesCount; panel++) {
        for (int knob = 0; knob < 4; knob++) {
            int intValue = (int)(knobValues[panel][knob] * 100);
            presetBuffer[idx++] = (uint32_t)intValue;
        }
    }
    
    // Retry on write failure (retry count = 1)
    uint8_t result;
    for (int retry = 0; retry < 2; retry++) { // 2 attempts = 1 retry
        result = BSP_SD_WriteBlocks(presetBuffer, PRESET_SECTOR, 1, 2000); // 2 second timeout
        if (result == MSD_OK) {
            SetDebugMessage("saved");
            return true;
        }
    }
    
    // SetDebugMessageF("save fail: %d", result);
    return false;
}

bool LoadPreset() {
    // Check card state before attempting to read
    uint8_t cardState = BSP_SD_GetCardState();
    if (cardState != MSD_OK) {
        // SetDebugMessageF("card not ready: %d", cardState);
        return false;
    }
    
    uint8_t result = BSP_SD_ReadBlocks(presetBuffer, PRESET_SECTOR, 1, PRESET_TIMEOUT_MS);
    if (result != MSD_OK) {
        // SetDebugMessageF("load fail: %d", result);
        return false;
    }
    
    // Optional: Check if preset appears to be valid (not all zeros)
    // This helps detect if the sector was never written to
    bool allZeros = true;
    for (int i = 0; i < PRESET_BUFFER_LENGTH; i++) {
        if (presetBuffer[i] != 0) {
            allZeros = false;
            break;
        }
    }
    if (allZeros) {
        // Preset sector is empty/uninitialized - treat as "no preset saved yet"
        return false;
    }
    
    // Load all knob normalizedValues (panelModesCount panels × 4 knobs)
    int idx = 0;
    for (int panel = 0; panel < panelModesCount; panel++) {
        for (int knob = 0; knob < 4; knob++) {
            int intValue = (int)presetBuffer[idx++];
            knobValues[panel][knob] = intValue / 100.0f;
        }
    }
    
    // SetDebugMessage("ld");
    return true;
}