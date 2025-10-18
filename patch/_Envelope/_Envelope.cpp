#include "daisy_patch.h"
#include "daisysp.h"
#include <algorithm>
#include <string>
#include <cmath>
#include <vector>
#include "midi/ShiftRegisterMidi.h"
#include "hid/parameter.h"
#include "tuning/ScalaTuning.h"
#include "tuning/TuningCalculator.h"
#include "ScreenUtils.h"

// Font aliases for cleaner code
#define font_s Font_6x8    // 6x8 pixels - small, good for labels and compact info
#define font_m Font_7x10   // 7x10 pixels - medium, good for panel names
#define font_l Font_11x18  // 11x18 pixels - large, good for emphasis

using namespace daisy;
using namespace daisysp;

DaisyPatch      hw;
Fm2             osc1, osc2;
Oscillator      pan, lfo1, lfo2, lfo3;
// Oscillator      voice1Osc, voice3Osc;  // Internal oscillators for voices 1 and 3
SdmmcHandler    sdcard;
FatFSInterface  fsi;

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

// Shift Register Mode (disabled for now)
bool shiftRegisterMode = false;

// Sequencer Mode
bool sequencerMode = false;
std::vector<uint8_t> sequencerNotes;  // Array of held notes for sequencer
bool sequencerNotesAscending = true;  // Note ordering direction
uint8_t sequencerNoteIndex = 0;  // Current index in sequencer notes array
float sequencerNoteLengthPercent = 0.5f;  // Note length as percentage of step (10%-90%)
bool sequencerUsingInitialCapture = false;  // True when using initially captured notes (don't stop on release)

// option to use internal oscillators for voices 1 and 3
bool useInternalOscillators = true;

// Tuning system variables
uint8_t currentTuningIndex = 0;  // Current tuning preset index
// this value currently corresponds to the 2 semitone pitch bend range configured on Intellijel
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

// Display update timing
uint32_t lastDisplayUpdate = 0;
const uint32_t DISPLAY_UPDATE_INTERVAL_MS = 100; // Update display every 100ms

// Trigger off timing
// int8_t currentNote = 0;
uint32_t triggerOffTime = 0;
const uint32_t TRIGGER_OFF_DELAY_MS = 50; // 10ms delay for trigger off
bool triggerOffPending = false;

// CC reset timing - for channel assignment on channel 16
const uint32_t CC_RESET_DELAY_MS = 200; // delay before resetting CC to lowest value
uint8_t lastCCValue = 0; // Track the last CC value sent (start with lowest CC value)

// CC Subdivision System variables
uint8_t ccSlotValues[8] = {0, 18, 36, 54, 73, 91, 109, 127}; // Equally distributed CC values 0-127
uint8_t ccSlotSubdivisions[8] = {1, 1, 1, 1, 1, 1, 1, 1}; // Default all subdivisions to 1
uint8_t ccSlotCounters[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Track note count for each slot
uint32_t globalNoteCounter = 0; // Increments on every note-on

// CC Queue System
struct CCQueueItem {
    uint8_t ccValue;
    uint32_t sendTime;
    uint32_t holdUntil; // Time when this CC should be released
    bool isReset; // true if this is a reset to lowest value
};

const size_t CC_QUEUE_SIZE = 16; // Maximum queue size
CCQueueItem ccQueue[CC_QUEUE_SIZE];
size_t ccQueueHead = 0;
size_t ccQueueTail = 0;
size_t ccQueueCount = 0;
uint32_t ccLatchTime = 0; // Time when CC was last sent
bool ccIsLatched = false; // Whether CC is currently latched to a value

// Encoder long press timing
const float ENCODER_SEQUENCER_TOGGLE_MS = 500.0f;
bool encoderWasPressed = false;
bool longPressHandled = false;

// MIDI Clock variables
const int32_t CLOCK_BPM_MIN = 10;     // Minimum BPM
const int32_t CLOCK_BPM_MAX = 1000;   // Maximum BPM
const int32_t CLOCK_BPM_DEFAULT = 120; // Default BPM
const int32_t CLOCK_BPM_INCREMENT = 10; // BPM change per encoder tick

int32_t clockBpm = CLOCK_BPM_DEFAULT;  // Current BPM
uint32_t lastClockTime = 0;
uint32_t clockInterval = 0;  // Calculated interval between clock messages
bool clockEnabled = true;
int8_t encoderIncrement = 0;  // Track encoder rotation

// Trigger Sequence Generator variables
const uint8_t TRIGGER_SEQUENCE_LENGTH = 16;  // 16-step sequence
bool triggerSequence[TRIGGER_SEQUENCE_LENGTH] = {false};  // Sequence pattern
uint8_t currentSequenceStep = 0;  // Current step in sequence
uint32_t lastSequenceStepTime = 0;  // Last time sequence step advanced
uint32_t sequenceStepInterval = 0;  // Interval between sequence steps
bool sequenceEnabled = true;  // Enable/disable sequence
uint8_t triggerNote = 36;  // MIDI note for triggers (C2)

// Trigger off timing for sequence
uint32_t sequenceTriggerOffTime = 0;
bool sequenceTriggerOffPending = false;

// Sequencer note-off timing
uint32_t sequencerNoteOffTime = 0;
bool sequencerNoteOffPending = false;
uint8_t sequencerNoteToTurnOff = 0;
int8_t sequencerVoiceToTurnOff = -1;

// Parameter objects for trigger sequence controls
Parameter densityParam, noteParam;

struct panelStruct
{
    std::string     name;
    std::string     input1Name;
    std::string     input2Name;
    std::string     input3Name;
    std::string     input4Name;
    float           values[4];
};
panelStruct displayPanels[6] = {
    { 
        name: "ADSR", 
        input1Name: "A", 
        input2Name: "D/R", 
        input3Name: "S",
        input4Name: "Min",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        name: "PANNING",
        input1Name: "Freq",
        input2Name: "Amp",
        input3Name: "",
        input4Name: "",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        name: "OSCILLATORS",
        input1Name: "Waveform",
        input2Name: "",
        input3Name: "",
        input4Name: "",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        name: "TRIGSEQ",
        input1Name: "Density",
        input2Name: "Order",
        input3Name: "Length",
        input4Name: "",
        values: {0.0f, 0.0f, 0.5f, 0.0f}
    },
    {
        name: "TUNING",
        input1Name: "Tuning",
        input2Name: "Range",
        input3Name: "MIDI",
        input4Name: "Osc",
        values: {0.0f, 0.0f, 1.0f, 1.0f}
    },
    {
        name: "CC SLOTS",
        input1Name: "CC1-2",
        input2Name: "CC3-4",
        input3Name: "CC5-6",
        input4Name: "CC7-8",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    }
};
int panelModesCount = sizeof(displayPanels) / sizeof(displayPanels[0]);
panelStruct currentPanel;
int noteCount = 0;

float previousKnobState [4];
float smoothedKnobState[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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
struct pluckStruct{
    PolyPluck<NUM_VOICES> synth;
    float wetDry; // param 1
    float decay;  // param 2
};
pluckStruct plucks[4];
// #define MAX_DELAY ((size_t)(10.0f * 48000.0f))
// 10 second delay line on the external SDRAM
// DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delay;

envStruct envelopes[4];
void      ProcessControls();
// void      UpdateEnvelopes();
void      ApplyVCAs();
void      ApplyPanning(float* data);
void      UpdateOled();
void      plucksApply();
void      InitPan(float samplerate);
void      SendPitchBend(uint8_t channel, int16_t bendValue);
void      ProcessCCSlots();
void      AddCCToQueue(uint8_t ccValue, bool isReset = false);
void      ProcessCCQueue();

// Trigger Sequence Generator functions
void      InitTriggerSequence();
void      UpdateTriggerSequence();
void      AdvanceSequenceStep();
void      GenerateEuclideanRhythm(int numTriggers, int numSteps, bool* pattern);
void      BuildPattern(int level, std::vector<bool>& result, const std::vector<int>& count, const std::vector<int>& remainder);

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

void DisplayMessage(const char* str)
{
    // Use fixed-width format to prevent overlap - avoid bottom row (y=56-63) reserved for general parameters
    // Position at y=48 to stay above bottom row
    WriteFixedString(hw, 0, 48, 11, font_s, str);  // Reduced width to 11 chars to stay left
    hw.display.Update();
}

void ClearPanelArea()
{
    // Clear the panel-specific area (y=24-55) to prevent overlap when switching panels
    // Reserve bottom row (y=56-63) for general parameters
    // Draw black rectangles to clear the area (false = black fill)
    hw.display.DrawRect(0, 24, 127, 55, false, true);  // Fill with black (false = black)
}

// void UpdateEnvelopes() {
//     float ctrl4 = hw.controls[3].Process(); // the fourth control knob controls baseline level
//     for(int j = 0; j < 4; j++)
//     {
//         envelopes[j].envSig = std::max(envelopes[j].env.Process(envelopes[j].gate), ctrl4);
//     }
// }

void PanEqualPowerStereo(float pan, float value, float* left, float* right)
{
    // Equal power panning: pan goes from -1 (full left) to +1 (full right)
    // Angle goes from 0 to π/2 as pan goes from -1 to +1
    float angle = (pan + 1.0f) * M_PI * 0.25f;
    *left       = value * cosf(angle);
    *right      = value * sinf(angle);
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

    data[0] = L1 + L2 + L3 + L4;
    data[1] = R1 + R2 + R3 + R4;
    hw.seed.dac.WriteValue(DacHandle::Channel::ONE, ((panOutput + 1.0f) / 2.0f) * 4095);
}

float IncrementTowards(float value, float target)
{
    float incrementUp = 0.01f;
    float incrementDown = 0.00001f;
    float increment = 0.0001f;
    if (value < target)
    {
        value += incrementUp;
        if (value > target)
        {
            value = target;
        }
    }
    else if (value > target)
    {
        value -= incrementDown;
        if (value < target)
        {
            value = target;
        }
    }
    return value;
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

// Apply VCA to inputs based on envelope values
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

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    ProcessControls();
    
    // Process envelopes at audio rate for consistent timing
    // float ctrl4 = hw.controls[3].Process(); // the fourth control knob controls baseline level

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

    for(size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            // results[j] = const_cast<float*>(&in[j][i]);
            results[j] = in[j][i];
        }

        // Use internal oscillators for voices 1 and 3 if enabled
        if (useInternalOscillators) {
            results[1] = voiceInterpOsc[1].Process();
            results[3] = voiceInterpOsc[3].Process();
        }

        // Panel 1
        ApplyVCAs(results);

        // Panel 2
        ApplyPanning(results);

        // plucksApply(results);

        // if (currentPanel.name == "Pluck") {
        // out[0][i] = sig;
        // out[1][i] = sig;
        // }
        out[0][i] = results[0];
        out[1][i] = results[1];
        
        // Output voices 0 and 2 oscillators to audio outputs 3 and 4 when enabled
        if (useInternalOscillators) {
            if (envelopes[0].gate) {
                out[2][i] = voiceInterpOsc[0].Process();
            } else {
                out[2][i] = 0.0f;
            }
            
            if (envelopes[2].gate) {
                out[3][i] = voiceInterpOsc[2].Process();
            } else {
                out[3][i] = 0.0f;
            }
        } else {
            // When internal oscillators are disabled, output silence on channels 3 and 4
            out[2][i] = 0.0f;
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
        
        // Set initial ADSR values - these will be updated by ProcessKnobs based on current panel
        // for some reason this is currently not working
        envelopes[i].env.SetTime(ADSR_SEG_ATTACK, 0.0001f);    // 1ms attack
        envelopes[i].env.SetTime(ADSR_SEG_DECAY, 2.5f);        // 500ms decay
        envelopes[i].env.SetTime(ADSR_SEG_RELEASE, 2.5f);       // 200ms release
        envelopes[i].env.SetSustainLevel(1.0f);                 // 100% sustain
    }
}

void PassthroughMidiMessage(MidiEvent m)
{
    int8_t channelOffset = 0; // probably don't need this
    if (m.channel > 4)
    {
        // return;
    }

    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();

            // Send pitch bend before note-on if tuning is enabled
            if (sendPitchBendMidi) {
                const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                float centsDeviation = CalculateCentsDeviation(p.note, tuning);
                int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                SendPitchBend(m.channel + channelOffset, pitchBendValue);
            }

            uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + m.channel + channelOffset), p.note, p.velocity};

            // if (m.channel == 0)
            // {
            //     bytes[0] = 0x90;
            //     // osc1.SetFrequency(mtof(p.note) / 4.0);
            // } else if (m.channel == 1)
            // {
            //     bytes[0] = 0x91;
            //     // osc2.SetFrequency(mtof(p.note) / 4.0);
            // }
            hw.midi.SendMessage(bytes, 3);
        }
        break;
        case NoteOff:
        {
            NoteOffEvent p = m.AsNoteOff();
            // for (int i = 0; i < 4; i++) {
            uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + m.channel + channelOffset), p.note, p.velocity};
                // hw.midi.SendMessage(bytes, 3);
            // }
            // if (m.channel == 0)
            // {
            //     bytes[0] = 0x80;
            // } else if (m.channel == 1)
            // {
            //     bytes[0] = 0x81;
            // }
            hw.midi.SendMessage(bytes, 3);
            // DisplayMessage("NoteOff");
        }
        break;
        case ControlChange:
        {
            ControlChangeEvent p = m.AsControlChange();
            switch(p.control_number)
            {
                case 76: // slide
                    // hw.seed.dac.WriteValue(DacHandle::Channel::ONE,
                    //     (p.value / 64.) * 4095);
                    
                    // CC 1 for cutoff.
                    // filt.SetFreq(mtof((float)p.value));
                    break;
                case 2:
                    // CC 2 for res.
                    // filt.SetRes(((float)p.value / 127.0f));
                    break;
                default: break;
            }
        }
        default: break;
    }
}

void SendMidiMesssage(uint8_t value, uint8_t channel, char* type)
{
    if (strcmp(type, "NOTE_ON") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + channel), value, 127};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "NOTE_OFF") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + channel), value, 0};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "TRIGGER_ON") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + channel), value, 127};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "TRIGGER_OFF") == 0)
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + channel), value, 0};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (strcmp(type, "CC") == 0)
    {
        uint8_t controller = 3; // this is configured in Intellijel 1U
        uint8_t bytes[3] = {static_cast<uint8_t>(0xB0 + channel), controller, value};
        hw.midi.SendMessage(bytes, 3);
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
    
    // Store current pitch bend value for this channel
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
    int8_t lowestNote = voices[0].note;
    for (int i = 1; i < 4; i++)
    {
        lowestNote = std::min(lowestNote, voices[i].note);
    }
    return lowestNote;
}

void HandleMidiMessage(MidiEvent m)
{   
    // Note: MIDI passthrough is now handled directly in voice allocation code
    // to send on the correct allocated voice channels
    
    // to handle round robin properly, may need to handle it here in Daisy, instead of using the setting
    // on MIDI 1U
    // int8_t channel = noteCount % 4;
    // int8_t channelOffset = 1; // MPE mode shifts this up one
    int8_t channelOffset = 0; // MPE mode shifts this up one

    // can use Daisy to send the highest MIDI note to Intellijel?

    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();
            
            if (shiftRegisterMode)
            {
                AddNoteToQueue(p.note, p.velocity);
                
                // Also send MIDI to external devices with tuning applied
                if (sendPitchBendMidi) {
                    const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                    float centsDeviation = CalculateCentsDeviation(p.note, tuning);
                    int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                    SendPitchBend(m.channel + channelOffset, pitchBendValue);
                }
                
                uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + m.channel + channelOffset), p.note, p.velocity};
                hw.midi.SendMessage(bytes, 3);
            } else if (sequencerMode) {
                // Sequencer mode: Add note to sequencer notes array
                AddNoteToSequencer(p.note);
                
                // Don't immediately allocate voices - sequencer will trigger them
                // Just update display
                // char message[60];
                // snprintf(message, 60, "Seq:%d Notes:%d", p.note, static_cast<int>(sequencerNotes.size()));
                // DisplayMessage(message);
            } else {
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
                                static_cast<uint8_t>(0x80 + voiceIndex + channelOffset), 
                                static_cast<uint8_t>(voices[voiceIndex].note), 
                                0
                            };
                            hw.midi.SendMessage(noteOffBytes, 3);
                        }
                    }
                }
                
                // Store the allocated voice channel
                p.channel = voiceIndex;
                
                // Advance round-robin index for next note
                nextVoiceIndex = (voiceIndex + 1) % 4;
                
                // Send pitch bend before note-on if tuning is enabled
                // Send on the allocated voice channel (voiceIndex)
                if (sendPitchBendMidi) {
                    const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                    float centsDeviation = CalculateCentsDeviation(p.note, tuning);
                    int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                    SendPitchBend(voiceIndex + channelOffset, pitchBendValue);
                }
                
                // Send MIDI note-on to external devices on the allocated voice channel
                uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + voiceIndex + channelOffset), p.note, p.velocity};
                hw.midi.SendMessage(bytes, 3);
                
                // Update internal oscillator frequencies BEFORE setting gate to avoid clicks
                if (useInternalOscillators) {
                    float freq = MidiNoteToFrequency(p.note, voiceIndex);
                    voiceInterpOsc[voiceIndex].SetFreq(freq);
                }
                
                // Update voice state
                envelopes[voiceIndex].gate = true;
                voices[voiceIndex].note = p.note;
                voices[voiceIndex].velocity = p.velocity;
                voices[voiceIndex].allocationOrder = voiceAllocationCounter++;
                
                // Set velocity-scaled sustain level before retriggering
                float sustainKnobValue = hw.controls[2].Process(); // Read sustain knob directly
                float baseSustainLevel = 0.01f * powf(100.0f, sustainKnobValue);
                float velocityFactor = p.velocity / 127.0f;
                float velocityScaledSustain = baseSustainLevel * velocityFactor;
                envelopes[voiceIndex].env.SetSustainLevel(velocityScaledSustain);
                
                envelopes[voiceIndex].env.Retrigger(true);
            }

            // pass highest currently held note to Intellijel via channel 16 and CC
            // 8 on the Intellijel Xpander
            currentHighestNote = getCurrentHighestNote();
            if (currentHighestNote != lastHighestNote) {
                SendMidiMesssage(currentHighestNote, 14, "NOTE_ON");
            }
            // turn off previous note
            if (lastHighestNote != 0 && lastHighestNote != currentHighestNote) {
                SendMidiMesssage(lastHighestNote, 14, "NOTE_OFF");
            }
            lastHighestNote = currentHighestNote;

            // pass lowest currently held note to Intellijel via channel 15 and CC
            // 7 on the Intellijel Xpander
            currentLowestNote = getCurrentLowestNote();
            if (currentLowestNote != lastLowestNote) {
                SendMidiMesssage(currentLowestNote, 13, "NOTE_ON");
            }
            // turn off previous note
            if (lastLowestNote != 0 && lastLowestNote != currentLowestNote) {
                SendMidiMesssage(lastLowestNote, 13, "NOTE_OFF");
            }
            lastLowestNote = currentLowestNote;

            // Turn off the previously played note first
            if (lastCurrentNote != 0) {
                SendMidiMesssage(lastCurrentNote, 15, "NOTE_OFF");
            }

            // this voice is meant to be sent to Multigrain
            // pass current note and trigger to Intellijel via channel 13
            lastCurrentNote = currentNote;
            currentNote = p.note;
            
            
            // Send the new note
            SendMidiMesssage(p.note, 15, "NOTE_ON");
            
            // Process CC slots based on subdivision logic
            ProcessCCSlots();
            
            // Send trigger on channel 15 after CC to ensure consumer sees updated CC value
            // Using a fixed trigger note (e.g., C3 = 60) for triggering
            SendMidiMesssage(60, 15, "TRIGGER_ON");
            
            // Set timer for trigger off after 10ms delay
            triggerOffTime = hw.seed.system.GetNow() + TRIGGER_OFF_DELAY_MS;
            triggerOffPending = true;

            // probably move outside of audio callback
            // char message[60];
            // snprintf(message, 60, "Note:%d Ch:%d", p.note, static_cast<int>(p.channel));
            // DisplayMessage(message);
            
            noteCount++;
        }
        break;
        case NoteOff:
        {
            NoteOffEvent p = m.AsNoteOff();
            
            if (shiftRegisterMode)
            {
                RemoveNoteFromQueue(p.note);
                
                // Also send note-off to external devices
                uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + m.channel + channelOffset), p.note, p.velocity};
                hw.midi.SendMessage(bytes, 3);
            } else if (sequencerMode) {
                // Sequencer mode: Remove note from sequencer notes array
                RemoveNoteFromSequencer(p.note);
                
                // Update display
                // char message[60];
                // snprintf(message, 60, "Seq Off:%d Notes:%d", p.note, static_cast<int>(sequencerNotes.size()));
                // DisplayMessage(message);
            } else {
                // Voice allocation: turn off all voices playing this note
                for (int i = 0; i < 4; i++) {
                    if (voices[i].note == p.note) {
                        // Send MIDI note-off to external devices on this voice's channel
                        uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + i + channelOffset), p.note, p.velocity};
                        hw.midi.SendMessage(bytes, 3);
                        
                        envelopes[i].gate = false;
                        
                        // Clear voice data when note is released
                        voices[i].note = 0;
                        voices[i].velocity = 0;
                        voices[i].allocationOrder = 0;
                    }
                }
            }
            
            // update highest and lowest notes when a note is released
            currentHighestNote = getCurrentHighestNote();
            if (currentHighestNote != lastHighestNote) {
                if (lastHighestNote != 0) {
                    SendMidiMesssage(lastHighestNote, 15, "NOTE_OFF");
                }
                if (currentHighestNote != 0) {
                    SendMidiMesssage(currentHighestNote, 15, "NOTE_ON");
                }
            }
            lastHighestNote = currentHighestNote;

            currentLowestNote = getCurrentLowestNote();
            if (currentLowestNote != lastLowestNote) {
                if (lastLowestNote != 0) {
                    SendMidiMesssage(lastLowestNote, 14, "NOTE_OFF");
                }
                if (currentLowestNote != 0) {
                    SendMidiMesssage(currentLowestNote, 14, "NOTE_ON");
                }
            }
            lastLowestNote = currentLowestNote;
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

    InitEnvelopes(samplerate);

    // Initialize display panel values with current knob positions
    for(int i = 0; i < 4; i++)
    {
        // displayPanels[0].values[i] = hw.controls[i].Process();
        // displayPanels[1].values[i] = hw.controls[i].Process();
        // displayPanels[2].values[i] = hw.controls[i].Process();
        // displayPanels[3].values[i] = hw.controls[i].Process();
        // displayPanels[4].values[i] = hw.controls[i].Process();
    }
    
    // Initialize panning panel with default values
    displayPanels[1].values[0] = 0.02f;  // Frequency (0.2Hz / 10Hz max = 0.02)
    displayPanels[1].values[1] = 1.0f;   // Amplitude (full effect)
    
    // Initialize tuning panel with default values
    displayPanels[4].values[0] = 0.0f;  // Tuning selector (12-TET)
    displayPanels[4].values[1] = 0.09f;  // Pitch bend range (200 cents)
    displayPanels[4].values[2] = 1.0f;  // MIDI output enabled
    displayPanels[4].values[3] = 1.0f;  // Internal oscillators enabled
    
    // Initialize pitch bend values to center (no bend)
    for (int i = 0; i < 16; i++) {
        currentPitchBendValues[i] = 8192;
    }

    panelMode = 0;
    currentPanel = displayPanels[panelMode];

    
    // Initialize clock interval
    clockInterval = static_cast<uint32_t>(60000 / (clockBpm * 24));
    lastClockTime = hw.seed.system.GetNow();
    
    // Initialize trigger sequence
    InitTriggerSequence();

    // Initialize sequencer mode
    sequencerNotes.clear();
    sequencerNoteIndex = 0;
    sequencerNotesAscending = true;
    sequencerNoteLengthPercent = 0.5f; // Default to 50%

    // Initialize parameter objects for trigger sequence controls
    densityParam.Init(hw.controls[0], 0.0f, 16.5f, Parameter::LINEAR); // maybe does not actually reach the maximum value
    noteParam.Init(hw.controls[1], 36.0f, 84.0f, Parameter::LINEAR);

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
    
    // Initialize interpolated oscillators for all 4 voices
    for (int i = 0; i < 4; i++) {
        voiceInterpOsc[i].Init(samplerate);
        voiceInterpOsc[i].SetFreq(440.0f);
        voiceInterpOsc[i].SetAmp(1.0f);
        voiceInterpOsc[i].SetWaveformParam(0.0f);  // Start with sine wave
    }

    // Initialize parameters with linear scaling
    densityParam.Init(hw.controls[0], 0.0f, 16.0f, Parameter::LINEAR);
    noteParam.Init(hw.controls[1], 36.0f, 84.0f, Parameter::LINEAR);

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
        
        // Check for trigger off timing
        if (triggerOffPending && currentTime >= triggerOffTime)
        {
            // Send trigger off on channel 15 (matches the trigger on sent earlier)
            SendMidiMesssage(60, 15, "TRIGGER_OFF");
            triggerOffPending = false;
        }
        
        // Process CC queue
        ProcessCCQueue();
        
        // Check for sequence trigger off timing
        if (sequenceTriggerOffPending && currentTime >= sequenceTriggerOffTime)
        {
            SendMidiMesssage(triggerNote, 15, "TRIGGER_OFF");
            sequenceTriggerOffPending = false;
        }
        
        // Check for sequencer note-off timing
        if (sequencerNoteOffPending && currentTime >= sequencerNoteOffTime)
        {
            if (sequencerVoiceToTurnOff >= 0) {
                // Send MIDI note-off to external devices
                uint8_t bytes[3] = {
                    static_cast<uint8_t>(0x80 + sequencerVoiceToTurnOff), 
                    sequencerNoteToTurnOff, 
                    0
                };
                hw.midi.SendMessage(bytes, 3);
                
                // Turn off the envelope gate
                envelopes[sequencerVoiceToTurnOff].gate = false;
                
                // Clear voice data
                voices[sequencerVoiceToTurnOff].note = 0;
                voices[sequencerVoiceToTurnOff].velocity = 0;
            }
            sequencerNoteOffPending = false;
        }

        // Check for MIDI clock timing
        if (clockEnabled && (currentTime - lastClockTime) >= clockInterval)
        {
            SendMidiClock();
            lastClockTime = currentTime;
        }

        // Update trigger sequence
        UpdateTriggerSequence();
    }
}

// Helper function to format parameter values based on panel context
std::string FormatParameterValue(const std::string& panelName, int paramIndex, float value)
{
    if (panelName == "ADSR") {
        switch(paramIndex) {
            case 0: // Attack time
                if (value < 0.01f) return "0ms";
                else if (value < 0.1f) return std::to_string(static_cast<int>(value * 1000)) + "ms";
                else return std::to_string(static_cast<int>(value * 10)) + "s";
            case 1: // Decay/Release time
                if (value < 0.01f) return "0ms";
                else if (value < 0.1f) return std::to_string(static_cast<int>(value * 1000)) + "ms";
                else return std::to_string(static_cast<int>(value * 10)) + "s";
            case 2: // Sustain level
                return std::to_string(static_cast<int>(value * 100)) + "%";
            case 3: // Minimum level
                return std::to_string(static_cast<int>(value * 100)) + "%";
            default: return "0%";
        }
    }
    else if (panelName == "PANNING") {
        switch(paramIndex) {
            case 0: // Frequency
                return std::to_string(static_cast<int>(value * 10)) + "Hz";
            case 1: // Amplitude
                return std::to_string(static_cast<int>(value * 100)) + "%";
            default: return "0%";
        }
    }
    else if (panelName == "OSCILLATORS") {
        switch(paramIndex) {
            case 0: // Waveform
                if (value < 0.25f) return "Sine";
                else if (value < 0.5f) return "Tri";
                else if (value < 0.75f) return "Sqr";
                else return "Saw";
            default: return "Sine";
        }
    }
    else if (panelName == "TRIGSEQ") {
        switch(paramIndex) {
            case 0: // Density
                return std::to_string(static_cast<int>(value * 16)) + "/16";
            case 1: // Order
                return value < 0.5f ? "ASC" : "DESC";
            case 2: // Length
                return std::to_string(static_cast<int>(value * 100)) + "%";
            default: return "0";
        }
    }
    else if (panelName == "TUNING") {
        switch(paramIndex) {
            case 0: // Tuning selector
                return std::to_string(static_cast<int>(value * 9)) + "/9";
            case 1: // Range
                return std::to_string(static_cast<int>(100 + value * 1100)) + "c";
            case 2: // MIDI
                return value > 0.5f ? "ON" : "OFF";
            case 3: // Osc
                return value > 0.5f ? "ON" : "OFF";
            default: return "OFF";
        }
    }
    else if (panelName == "CC SLOTS") {
        switch(paramIndex) {
            case 0: // CC1-2
                return value < 0.01f ? "OFF" : std::to_string(static_cast<int>(1.0f / value));
            case 1: // CC3-4
                return value < 0.01f ? "OFF" : std::to_string(static_cast<int>(1.0f / value));
            case 2: // CC5-6
                return value < 0.01f ? "OFF" : std::to_string(static_cast<int>(1.0f / value));
            case 3: // CC7-8
                return value < 0.01f ? "OFF" : std::to_string(static_cast<int>(1.0f / value));
            default: return "OFF";
        }
    }
    
    // Default fallback
    return std::to_string(static_cast<int>(value * 100)) + "%";
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

void UpdateOled()
{
    // hw.display.Fill(false);

    // Draw vertical panel name bar (x=0-15)
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
    int paramValueY = 12;  // Y position for parameter value labels
    
    int knobPositions[4];
    CalculateKnobPositions(knobWidth, knobPadding, knobPositions);
    
    // knob input labels at the top
    int labelY = 0;  // Position at very top
    WriteFixedString(hw, knobPositions[0], labelY, 5, font_s, currentPanel.input1Name.c_str());
    WriteFixedString(hw, knobPositions[1], labelY, 5, font_s, currentPanel.input2Name.c_str());
    WriteFixedString(hw, knobPositions[2], labelY, 5, font_s, currentPanel.input3Name.c_str());
    WriteFixedString(hw, knobPositions[3], labelY, 5, font_s, currentPanel.input4Name.c_str());
    
    // Draw horizontal meters below labels
    int meterY = 8;  // Position below labels
    int maxMeterWidth = 22;  // Maximum meter width in pixels (fits within 25 pixel knob width)

    for (int i = 0; i < 4; i++)
    {
        // Clear the meter area first (draw black line to erase previous meter)
        hw.display.DrawLine(knobPositions[i], meterY, knobPositions[i] + maxMeterWidth, meterY, false);
        
        // Draw the stored panel value (not current knob position)
        float val = displayPanels[panelMode].values[i];
        int meterWidth = static_cast<int>(val * maxMeterWidth);  // Scale 0.0-1.0 to 0-22 pixels
        meterWidth = std::max(0, std::min(meterWidth, maxMeterWidth));  // Clamp to 0-22 range
        
        if (meterWidth > 0) {
            hw.display.DrawLine(knobPositions[i], meterY, knobPositions[i] + meterWidth, meterY, true);
        }
    }
    
    // Display parameter values below meters
    for (int i = 0; i < 4; i++) {
        std::string paramValue = FormatParameterValue(currentPanel.name, i, currentPanel.values[i]);
        WriteFixedString(hw, knobPositions[i], paramValueY, 5, font_s, paramValue.c_str());
    }
    
    // Show trigger sequence pattern when in TRIGSEQ mode
    if (currentPanel.name == "TRIGSEQ") {
        // Show current mode - REMOVE (now in bottom row)
        
        // Show step counter with fixed width - move to avoid bottom-right area
        WriteFixedStringF(hw, knobPositions[0], 24, 8, font_s, "Step:%02d", currentSequenceStep);

        // Show density value with fixed width - move to avoid bottom-right area
        int numTriggers = 0;
        for (int i = 0; i < TRIGGER_SEQUENCE_LENGTH; i++) {
            if (triggerSequence[i]) numTriggers++;
        }
        WriteFixedStringF(hw, knobPositions[2], 24, 5, font_s, "D:%02d", numTriggers);
        
        // Show sequence pattern as dots in one fixed-width string
        char patternStr[TRIGGER_SEQUENCE_LENGTH + 1];
        for (int i = 0; i < TRIGGER_SEQUENCE_LENGTH; i++) {
            patternStr[i] = triggerSequence[i] ? '*' : '-';
        }
        patternStr[TRIGGER_SEQUENCE_LENGTH] = '\0';
        WriteFixedString(hw, knobPositions[0], 32, TRIGGER_SEQUENCE_LENGTH, font_s, patternStr);
        
        // Show sequencer-specific information
        if (sequencerMode) {
            // Show note ordering direction - move to avoid conflicts
            WriteFixedString(hw, knobPositions[3], 32, 4, font_s, sequencerNotesAscending ? "ASC" : "DESC");
            
            // Show number of held notes - move to avoid conflicts
            WriteFixedStringF(hw, knobPositions[0], 40, 4, font_s, "N:%d", static_cast<int>(sequencerNotes.size()));
            
            // Show note length percentage (10%-80% range) - REMOVE (conflicts with bottom-right)
            
            // Show current sequencer note index if there are notes - REMOVE (conflicts with bottom-right)
        }
    }
    // Show tuning information when in TUNING mode
    else if (currentPanel.name == "TUNING") {
        // Show current tuning name with fixed width (21 chars to fill display width)
        const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
        WriteFixedString(hw, knobPositions[0], 24, 21, font_s, tuning->name);
        
        // Show pitch bend range with fixed width
        WriteFixedStringF(hw, knobPositions[0], 32, 10, font_s, "Rng:%4.0fc", pitchBendRange);
        
        // Show MIDI and Osc status with fixed width
        WriteFixedStringF(hw, knobPositions[3], 40, 2, font_s, "%s%s", 
                         sendPitchBendMidi ? "M" : "-", 
                         applyToInternalOsc ? "O" : "-");
    }
    // Show CC Slots information when in CC SLOTS mode
    else if (currentPanel.name == "CC SLOTS") {
        // Show current subdivisions for each pair
        WriteFixedStringF(hw, knobPositions[0], 24, 6, font_s, "1-2:%s", ccSlotSubdivisions[0] == 255 ? "OFF" : std::to_string(ccSlotSubdivisions[0]).c_str());
        WriteFixedStringF(hw, knobPositions[1], 24, 6, font_s, "3-4:%s", ccSlotSubdivisions[2] == 255 ? "OFF" : std::to_string(ccSlotSubdivisions[2]).c_str());
        WriteFixedStringF(hw, knobPositions[2], 24, 6, font_s, "5-6:%s", ccSlotSubdivisions[4] == 255 ? "OFF" : std::to_string(ccSlotSubdivisions[4]).c_str());
        WriteFixedStringF(hw, knobPositions[3], 24, 6, font_s, "7-8:%s", ccSlotSubdivisions[6] == 255 ? "OFF" : std::to_string(ccSlotSubdivisions[6]).c_str());
        
        // Show global note counter and queue status - move to avoid bottom-right area
        WriteFixedStringF(hw, knobPositions[0], 32, 8, font_s, "Note:%d", globalNoteCounter);
        WriteFixedStringF(hw, knobPositions[2], 32, 4, font_s, "Q:%d", ccQueueCount);
    }
    
    // === BOTTOM ROW: General State Info (always visible) ===
    // Use entire bottom row (y=56-63) for general parameters
    
    // Display current note and BPM in compact format: "60|120" - left side
    WriteFixedStringF(hw, knobPositions[0], 56, 10, font_s, "%3d|%3d", currentNote, clockBpm);

    // Display mode indicators - right side with proper spacing
    if (sequencerMode) {
        // Sequencer mode active
        WriteFixedString(hw, knobPositions[3], 56, 2, font_s, "SQ");
    } else {
        // Sequencer mode inactive - clear the area
        WriteFixedString(hw, knobPositions[3], 56, 5, font_s, "     ");  // 5 spaces to clear
    }
    
    // draw current knob values
    for (int i = 0; i < 4; i++)
    {
        // hw.display.SetCursor(0 + (i * 20), 25);
        float val = currentPanel.values[i];

        // bug: val oscillates between 0 and the actual value???
        // currently this seems to only get called when it is incorrectly reading 0 ... ?
        // if (val > 0.01f){
            // char printme[50];
            // snprintf(printme, sizeof(printme), "val: %d", static_cast<int>(val * 200));
            // DisplayMessage(printme);
            // int rectHeight = static_cast<int>(val * 20);
            // hw.display.DrawRect(i * 20, 40, i * 20 + 10, 40 - rectHeight, true, true);
        // }


        // hw.display.Update();
        // str = std::to_string(currentPanel.values[i]);
        // cstr = &str[0];
        // hw.display.WriteString(cstr, Font_6x8, true);
        // str = std::to_string(currentPanel.values[i]);
        // cstr = &str[0];
        // hw.display.WriteString("a", Font_6x8, true);
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
        // Adjust BPM by configured increment for each encoder tick
        clockBpm += encoderValue * CLOCK_BPM_INCREMENT;

        // Clamp BPM to configured range
        if (clockBpm < CLOCK_BPM_MIN) clockBpm = CLOCK_BPM_MIN;
        if (clockBpm > CLOCK_BPM_MAX) clockBpm = CLOCK_BPM_MAX;

        // Recalculate clock interval
        // MIDI clock sends 24 pulses per quarter note
        // Interval = 60000ms / (BPM * 24)
        clockInterval = static_cast<uint32_t>(60000 / (clockBpm * 24));
        
        // Recalculate sequence step interval
        // Each sequence step = 1/16th note = 6 MIDI clock pulses
        // Interval = 60000ms / (BPM * 4) for 16th note timing
        sequenceStepInterval = static_cast<uint32_t>(60000 / (clockBpm * 4));
        
        knobChanged = true; // Trigger display update

    }
    
    // Detect long press: trigger when held >= 0.5 seconds
    if(encoderPressed && timeHeld >= ENCODER_SEQUENCER_TOGGLE_MS && !longPressHandled)
    {
        // Long press detected - toggle sequencer mode
        sequencerMode = !sequencerMode;
        longPressHandled = true;
        
        if (sequencerMode) {
            // When enabling sequencer mode, capture currently held notes
            CaptureCurrentlyHeldNotes();
        } else {
            // Clear sequencer notes when disabling sequencer mode to prevent artifacts
            ClearSequencerNotes();
        }
        
        // Update display to show mode change
        UpdateOled();
    }
    else if(!encoderPressed && encoderWasPressed)
    {
        // Button was released
        if(!longPressHandled)
        {
            // Short press - cycle through panel modes
            panelMode = (panelMode + 1) % panelModesCount;
            currentPanel = displayPanels[panelMode];
            
            // Clear panel area to prevent overlap when switching panels
            ClearPanelArea();
            
            // Update current panel values to reflect current knob positions
            for (int i = 0; i < 4; i++)
            {
                currentPanel.values[i] = hw.controls[i].Process();
            }
            
            UpdateOled();
        }
        
        // Reset long press flag for next press
        longPressHandled = false;
    }
    
    encoderWasPressed = encoderPressed;
}

void ProcessKnobs()
{
    float inputs[4];
    int8_t inputIndex = -1; // assuming only one knob changes at a time
    float knobThreshold = 0.0001; // lower values for slower movement
    float alpha = 0.25; // higher value for less smoothing

    for (int i = 0; i < 4; i++)
    {
        inputs[i] = hw.controls[i].Process();
        
        // Apply exponential moving average filter
        smoothedKnobState[i] = (alpha * inputs[i]) + ((1.0f - alpha) * smoothedKnobState[i]);
        
        // Compare smoothed values against threshold
        if (fabs(smoothedKnobState[i] - previousKnobState[i]) > knobThreshold)
        {
            inputIndex = i;
            // if (inputs[i] > 0.1f) {
                currentPanel.values[i] = inputs[i];
                displayPanels[panelMode].values[i] = inputs[i]; // Also update stored panel values
                knobChanged = true;
            // }
        }
    }

    if (inputIndex > -1) {
        // float val = inputs[inputIndex];
        // currentPanel.values[inputIndex] = val;
        // std::string str = std::to_string(inputs[inputIndex]);
        // DisplayMessage(str.c_str());

        // float val = 0.141414f;
        // char printme[50];
        // snprintf(printme, sizeof(printme), "val: %d", static_cast<int>(val * 1000));
        // DisplayMessage(printme);

        // hw.display.DrawRect(0, 0, 128, 64, true);
        // hw.display.SetCursor(35, 0);
        // str = currentPanel.input2Name;
        // hw.display.WriteString(cstr, Font_6x8, true);
    }

    if (currentPanel.name == "ADSR")
    {
        for (int i = 0; i < 4; i++)
        {
            switch(inputIndex)
            {
                case 0:
                    // Attack: logarithmic scaling 0.0001s to 5.0s
                    {
                        float attackTime = 0.0001f * powf(5000.0f, inputs[0]);
                        envelopes[i].env.SetTime(ADSR_SEG_ATTACK, attackTime);
                    }
                    break;
                case 1:
                    // Decay/Release: logarithmic scaling 0.0001s to 3.0s
                    {
                        float decayTime = 0.0001f * powf(3000.0f, inputs[1]);
                        envelopes[i].env.SetTime(ADSR_SEG_DECAY, decayTime);
                        envelopes[i].env.SetTime(ADSR_SEG_RELEASE, decayTime);
                    }
                    break;
                case 2:
                    // Sustain: logarithmic scaling 0.01 to 1.0
                    {
                        float sustainLevel = 0.01f * powf(100.0f, inputs[2]);
                        envelopes[i].env.SetSustainLevel(sustainLevel);
                    }
                    break;
                case 3:
                    // Minimum level (used in envelope processing)
                    voicesMinLevel = inputs[3];
                    break;
                default:
                    break;
            }
        }
    }
    else if (currentPanel.name == "PANNING")
    {
        switch(inputIndex)
        {
            case 0:
                // Update manual pan frequency: 0 to 10Hz range
                panFreq = inputs[0] * 10.0f;
                break;
            case 1:
                // Pan amplitude control: 0 = all voices centered, 1 = full panning effect
                panAmp = inputs[1];
                break;
            default:
                break;
        }
    }
    else if (currentPanel.name == "Pluck")
    {
        for (size_t j = 0; j < 4; j++)
        {
            plucks[j].wetDry = inputs[0];
            // plucks[j].decay = inputs[1];
        }
        
        // hw.controls[0].Process();
        // ProcessPluck();
    }
    else if (currentPanel.name == "OSCILLATORS")
    {
        switch(inputIndex)
        {
            case 0:
                // Waveform control: 0.0 = sine, 0.33 = triangle, 0.66 = square, 1.0 = saw
                for (int i = 0; i < 4; i++) {
                    voiceInterpOsc[i].SetWaveformParam(inputs[0]);
                }
                break;
            default:
                break;
        }
    }
    else if (currentPanel.name == "TRIGSEQ")
    {
        // Process parameters continuously
        sequenceEnabled = sequencerMode; // Enable sequence when sequencer mode is active
        
        // Only regenerate pattern when density knob changes
        if (inputIndex == 0) {
            // Round the parameter value to ensure we get exact integer values
            int numTriggers = static_cast<int>(densityParam.Process() + 0.5f);
            numTriggers = std::max(0, std::min(numTriggers, static_cast<int>(TRIGGER_SEQUENCE_LENGTH)));
            
            // Generate Euclidean rhythm pattern
            GenerateEuclideanRhythm(numTriggers, TRIGGER_SEQUENCE_LENGTH, triggerSequence);
            knobChanged = true;
        }
        
        // Handle knob 2: Note ordering control
        if (inputIndex == 1) {
            bool newAscending = inputs[1] < 0.5f;
            if (newAscending != sequencerNotesAscending) {
                sequencerNotesAscending = newAscending;
                SortSequencerNotes(); // Re-sort existing notes
                knobChanged = true;
            }
        }
        
        // Handle knob 4: Note length control (10% to 80% of step duration)
        if (inputIndex == 3) {
            float newLengthPercent = 0.1f + inputs[3] * 0.8f; // Map 0-1 to 0.1-0.8
            if (fabs(newLengthPercent - sequencerNoteLengthPercent) > 0.01f) {
                sequencerNoteLengthPercent = newLengthPercent;
                knobChanged = true;
            }
        }
    }
    else if (currentPanel.name == "TUNING")
    {
        switch(inputIndex)
        {
            case 0:
                // Tuning selector: map knob value to tuning index
                {
                    float tuningValue = inputs[0];
                    uint8_t newTuningIndex = static_cast<uint8_t>(tuningValue * (NUM_TUNING_PRESETS - 1) + 0.5f);
                    if (newTuningIndex != currentTuningIndex) {
                        currentTuningIndex = newTuningIndex;
                        knobChanged = true;
                        // Apply tuning changes to sequencer notes
                        ApplyTuningToSequencerNotes();
                    }
                }
                break;
            case 1:
                // Pitch bend range: ±100 to ±1200 cents
                {
                    float rangeValue = inputs[1];
                    float newRange = 100.0f + rangeValue * 1100.0f; // 100 to 1200 cents
                    if (fabs(newRange - pitchBendRange) > 1.0f) {
                        pitchBendRange = newRange;
                        knobChanged = true;
                        // Apply tuning changes to sequencer notes
                        ApplyTuningToSequencerNotes();
                    }
                }
                break;
            case 2:
                // Enable/disable pitch bend MIDI output
                {
                    bool newSendMidi = inputs[2] > 0.5f;
                    if (newSendMidi != sendPitchBendMidi) {
                        sendPitchBendMidi = newSendMidi;
                        knobChanged = true;
                    }
                }
                break;
            case 3:
                // Enable/disable internal oscillator tuning
                {
                    bool newApplyOsc = inputs[3] > 0.5f;
                    if (newApplyOsc != applyToInternalOsc) {
                        applyToInternalOsc = newApplyOsc;
                        knobChanged = true;
                        // Apply tuning changes to sequencer notes
                        ApplyTuningToSequencerNotes();
                    }
                }
                break;
            default:
                break;
        }
    }
    else if (currentPanel.name == "CC SLOTS")
    {
        switch(inputIndex)
        {
            case 0:
                // CC1-2 subdivision control: 0 = never (255), 1 = always (1)
                {
                    float knobValue = inputs[0];
                    uint8_t subdivision;
                    if (knobValue < 0.01f) subdivision = 255; // Never send (infinity)
                    else subdivision = static_cast<uint8_t>(1.0f / knobValue); // Map 0.01-1.0 to 100-1
                    
                    ccSlotSubdivisions[0] = subdivision;
                    ccSlotSubdivisions[1] = subdivision;
                    knobChanged = true;
                }
                break;
            case 1:
                // CC3-4 subdivision control: 0 = never (255), 1 = always (1)
                {
                    float knobValue = inputs[1];
                    uint8_t subdivision;
                    if (knobValue < 0.01f) subdivision = 255; // Never send (infinity)
                    else subdivision = static_cast<uint8_t>(1.0f / knobValue); // Map 0.01-1.0 to 100-1
                    
                    ccSlotSubdivisions[2] = subdivision;
                    ccSlotSubdivisions[3] = subdivision;
                    knobChanged = true;
                }
                break;
            case 2:
                // CC5-6 subdivision control: 0 = never (255), 1 = always (1)
                {
                    float knobValue = inputs[2];
                    uint8_t subdivision;
                    if (knobValue < 0.01f) subdivision = 255; // Never send (infinity)
                    else subdivision = static_cast<uint8_t>(1.0f / knobValue); // Map 0.01-1.0 to 100-1
                    
                    ccSlotSubdivisions[4] = subdivision;
                    ccSlotSubdivisions[5] = subdivision;
                    knobChanged = true;
                }
                break;
            case 3:
                // CC7-8 subdivision control: 0 = never (255), 1 = always (1)
                {
                    float knobValue = inputs[3];
                    uint8_t subdivision;
                    if (knobValue < 0.01f) subdivision = 255; // Never send (infinity)
                    else subdivision = static_cast<uint8_t>(1.0f / knobValue); // Map 0.01-1.0 to 100-1
                    
                    ccSlotSubdivisions[6] = subdivision;
                    ccSlotSubdivisions[7] = subdivision;
                    knobChanged = true;
                }
                break;
            default:
                break;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        previousKnobState[i] = smoothedKnobState[i]; // Update with smoothed values for next comparison
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
        
        wet = plucks[i].wetDry;
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
        sig = plucks[i].synth.Process(trig, note);

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
            if (useInternalOscillators) {
                float freq = MidiNoteToFrequency(static_cast<int8_t>(state.note), static_cast<int8_t>(i));
                voiceInterpOsc[i].SetFreq(freq);
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
    for (size_t i = 0; i < sequencerNotes.size(); i++) {
        if (sequencerNotes[i] == note) {
            return; // Note already exists, don't add duplicate
        }
    }
    
    // Add note to array
    sequencerNotes.push_back(note);
    
    // Sort notes based on current ordering preference
    SortSequencerNotes();
    
    // Reset sequencer note index when new notes are added
    sequencerNoteIndex = 0;
    
    // If we're adding notes after initial capture, reset the flag
    // This means sequencer will now stop when notes are released (normal behavior)
    if (sequencerUsingInitialCapture) {
        sequencerUsingInitialCapture = false;
    }
}

void RemoveNoteFromSequencer(uint8_t note)
{
    // If we're using initially captured notes, preserve all notes in the sequencer
    // This prevents losing notes when they're released at slightly different times
    if (sequencerUsingInitialCapture) {
        return; // Don't remove any notes when using initially captured notes
    }
    
    // Find and remove the note (normal behavior for later-added notes)
    for (auto it = sequencerNotes.begin(); it != sequencerNotes.end(); ++it) {
        if (*it == note) {
            sequencerNotes.erase(it);
            break;
        }
    }
    
    // Adjust sequencer note index if needed
    if (!sequencerNotes.empty() && sequencerNoteIndex >= sequencerNotes.size()) {
        sequencerNoteIndex = 0;
    }
}

void SortSequencerNotes()
{
    if (sequencerNotesAscending) {
        std::sort(sequencerNotes.begin(), sequencerNotes.end());
    } else {
        std::sort(sequencerNotes.begin(), sequencerNotes.end(), std::greater<uint8_t>());
    }
}

void ClearSequencerNotes()
{
    sequencerNotes.clear();
    sequencerNoteIndex = 0;
    sequencerUsingInitialCapture = false;  // Reset flag when clearing notes
}

void CaptureCurrentlyHeldNotes()
{
    // Clear existing sequencer notes first
    sequencerNotes.clear();
    
    // Capture all currently held notes from the voices array
    for (int i = 0; i < 4; i++) {
        if (envelopes[i].gate && voices[i].note > 0) {
            // Add note to sequencer if it's currently held
            AddNoteToSequencer(static_cast<uint8_t>(voices[i].note));
        }
    }
    
    // Reset sequencer note index to start from the beginning
    sequencerNoteIndex = 0;
    
    // Set flag to indicate we're using initially captured notes
    // This means sequencer won't stop when these notes are released
    sequencerUsingInitialCapture = true;
}

void ApplyTuningToSequencerNotes()
{
    // Apply tuning changes to all currently playing sequencer voices
    // This ensures that when tuning changes, the sequencer notes reflect the new tuning
    if (!sequencerMode || sequencerNotes.empty()) {
        return;
    }
    
    // Update pitch bend for all currently active voices that are playing sequencer notes
    for (int i = 0; i < 4; i++) {
        if (envelopes[i].gate && voices[i].note > 0) {
            // Check if this voice is playing a note from the sequencer
            bool isSequencerNote = false;
            for (size_t j = 0; j < sequencerNotes.size(); j++) {
                if (voices[i].note == sequencerNotes[j]) {
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
            if (useInternalOscillators) {
                float freq = MidiNoteToFrequency(voices[i].note, static_cast<int8_t>(i));
                voiceInterpOsc[i].SetFreq(freq);
            }
        }
    }
}

// Trigger Sequence Generator Implementation
void InitTriggerSequence()
{
    // Initialize sequence with Euclidean rhythm (4 triggers out of 16 steps)
    GenerateEuclideanRhythm(4, TRIGGER_SEQUENCE_LENGTH, triggerSequence);

    // Initialize timing
    currentSequenceStep = 0;
    lastSequenceStepTime = hw.seed.system.GetNow();
    sequenceStepInterval = static_cast<uint32_t>(60000 / (clockBpm * 4));  // 16th note timing
}

void UpdateTriggerSequence()
{
    // Only run sequencer when sequencer mode is enabled
    if (!sequencerMode) {
        return;
    }
    
    if (!sequenceEnabled) {
        return;
    }

    uint32_t currentTime = hw.seed.system.GetNow();

    // Check if it's time to advance to the next sequence step
    if ((currentTime - lastSequenceStepTime) >= sequenceStepInterval) {
        AdvanceSequenceStep();
        lastSequenceStepTime = currentTime;
    }
}

void AdvanceSequenceStep()
{
    // Check if current step should trigger
    if (triggerSequence[currentSequenceStep]) {
        if (sequencerMode && !sequencerNotes.empty()) {
            // Sequencer mode: trigger next note from sequencer notes array
            uint8_t noteToTrigger = sequencerNotes[sequencerNoteIndex];
            
            // Voice allocation: Round-robin distribution across voices 0-3
            // In sequencer mode, we always use round-robin regardless of gate state
            // because note-offs are scheduled with delays and gates may still be true
            // This ensures proper voice distribution instead of always stealing the same voice
            int8_t voiceIndex = nextVoiceIndex;
            
            // Send note-off for the voice we're about to steal (if it's playing a note)
            if (voices[voiceIndex].note > 0) {
                uint8_t noteOffBytes[3] = {
                    static_cast<uint8_t>(0x80 + voiceIndex), 
                    static_cast<uint8_t>(voices[voiceIndex].note), 
                    0
                };
                hw.midi.SendMessage(noteOffBytes, 3);
            }
            
            // Advance round-robin index for next note
            nextVoiceIndex = (voiceIndex + 1) % 4;
            
            // Send pitch bend before note-on if tuning is enabled
            if (sendPitchBendMidi) {
                const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
                float centsDeviation = CalculateCentsDeviation(noteToTrigger, tuning);
                int16_t pitchBendValue = CentsToPitchBend(centsDeviation, pitchBendRange);
                SendPitchBend(voiceIndex, pitchBendValue);
            }
            
            // Send MIDI note-on to external devices on the allocated voice channel
            uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + voiceIndex), noteToTrigger, 127};
            hw.midi.SendMessage(bytes, 3);
            
            // Update internal oscillator frequencies BEFORE setting gate to avoid clicks
            if (useInternalOscillators) {
                float freq = MidiNoteToFrequency(noteToTrigger, voiceIndex);
                voiceInterpOsc[voiceIndex].SetFreq(freq);
            }
            
            // Update voice state
            envelopes[voiceIndex].gate = true;
            voices[voiceIndex].note = noteToTrigger;
            voices[voiceIndex].velocity = 127;
            voices[voiceIndex].allocationOrder = voiceAllocationCounter++;
            
            // Set velocity-scaled sustain level before retriggering
            float sustainKnobValue = hw.controls[2].Process(); // Read sustain knob directly
            float baseSustainLevel = 0.01f * powf(100.0f, sustainKnobValue);
            float velocityScaledSustain = baseSustainLevel; // Use full velocity for sequencer
            envelopes[voiceIndex].env.SetSustainLevel(velocityScaledSustain);
            
            envelopes[voiceIndex].env.Retrigger(true);
            
            // Process CC slots based on subdivision logic
            ProcessCCSlots();
            
            // Advance to next note in sequencer array
            sequencerNoteIndex = (sequencerNoteIndex + 1) % sequencerNotes.size();
            
            // Send trigger on channel 15 after CC to ensure consumer sees updated CC value
            SendMidiMesssage(60, 15, "TRIGGER_ON");
            
            // Set timer for trigger off after 10ms delay
            triggerOffTime = hw.seed.system.GetNow() + TRIGGER_OFF_DELAY_MS;
            triggerOffPending = true;
            
            // Schedule note-off based on note length percentage
            // Ensure note-off happens well before next step to avoid timing conflicts
            uint32_t noteOffDelay = static_cast<uint32_t>(sequenceStepInterval * sequencerNoteLengthPercent);
            noteOffDelay = std::min(noteOffDelay, sequenceStepInterval - 10); // Leave at least 10ms before next step
            sequencerNoteOffTime = hw.seed.system.GetNow() + noteOffDelay;
            sequencerNoteOffPending = true;
            sequencerNoteToTurnOff = noteToTrigger;
            sequencerVoiceToTurnOff = voiceIndex;
            
            // Display current sequencer note
            // char message[60];
            // snprintf(message, 60, "Seq:%d Ch:%d", noteToTrigger, voiceIndex);
            // DisplayMessage(message);
        } else {
            // Original trigger behavior for keyboard mode or when no sequencer notes
            SendMidiMesssage(127, 15, "CC");
            SendMidiMesssage(triggerNote, 15, "TRIGGER_ON");

            // Schedule trigger off after a short duration (50ms)
            sequenceTriggerOffTime = hw.seed.system.GetNow() + 50;
            sequenceTriggerOffPending = true;
        }
    }
    
    // Advance to next step
    currentSequenceStep = (currentSequenceStep + 1) % TRIGGER_SEQUENCE_LENGTH;
}

void AddCCToQueue(uint8_t ccValue, bool isReset)
{
    // Check if queue is full
    if (ccQueueCount >= CC_QUEUE_SIZE) {
        return; // Queue is full, drop the CC
    }
    
    // Calculate send time based on current time and delay
    uint32_t sendTime = hw.seed.system.GetNow();
    if (!isReset) {
        // For regular CC, send immediately
        sendTime += 10; // Small delay to ensure ordering
    } else {
        // For reset CC, use the configured delay
        sendTime += CC_RESET_DELAY_MS;
    }
    
    // Calculate hold duration - each CC is held for CC_RESET_DELAY_MS
    uint32_t holdUntil = sendTime + CC_RESET_DELAY_MS;
    
    // Add to queue
    ccQueue[ccQueueTail].ccValue = ccValue;
    ccQueue[ccQueueTail].sendTime = sendTime;
    ccQueue[ccQueueTail].holdUntil = holdUntil;
    ccQueue[ccQueueTail].isReset = isReset;
    
    ccQueueTail = (ccQueueTail + 1) % CC_QUEUE_SIZE;
    ccQueueCount++;
}

void ProcessCCQueue()
{
    uint32_t currentTime = hw.seed.system.GetNow();
    
    // Process all ready items in the queue
    while (ccQueueCount > 0) {
        CCQueueItem& item = ccQueue[ccQueueHead];
        
        // Check if it's time to send this CC
        if (currentTime >= item.sendTime) {
            // Send the CC
            SendMidiMesssage(item.ccValue, 15, "CC");
            lastCCValue = item.ccValue;
            ccLatchTime = currentTime;
            ccIsLatched = true;
            
            // Remove from queue
            ccQueueHead = (ccQueueHead + 1) % CC_QUEUE_SIZE;
            ccQueueCount--;
        } else {
            // Not time yet, stop processing
            break;
        }
    }
    
    // Check if CC should be latched down to lowest value
    if (ccIsLatched && ccQueueCount == 0) {
        // No more CCs in queue, check if we should latch down
        uint32_t timeSinceLastCC = currentTime - ccLatchTime;
        if (timeSinceLastCC >= CC_RESET_DELAY_MS) {
            // Latch down to lowest value (0)
            SendMidiMesssage(ccSlotValues[0], 15, "CC");
            lastCCValue = ccSlotValues[0];
            ccIsLatched = false;
        }
    }
}

void ProcessCCSlots()
{
    // Increment global note counter
    globalNoteCounter++;
    
    // Check all 8 CC slots and queue all matching ones
    for (int i = 0; i < 8; i++) {
        // Skip slots with subdivision 255 (never send)
        if (ccSlotSubdivisions[i] == 255) continue;
        
        // Check if this slot should trigger based on subdivision
        if (globalNoteCounter % ccSlotSubdivisions[i] == 0) {
            uint8_t ccValue = ccSlotValues[i];
            
            // Queue CC for all matching subdivisions
            AddCCToQueue(ccValue, false);
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