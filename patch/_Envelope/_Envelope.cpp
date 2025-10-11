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

using namespace daisy;
using namespace daisysp;

DaisyPatch      hw;
Fm2             osc1, osc2;
Oscillator      pan, lfo1, lfo2, lfo3;
// Oscillator      voice1Osc, voice3Osc;  // Internal oscillators for voices 1 and 3
SdmmcHandler    sdcard;
FatFSInterface  fsi;
WavPlayer       sampler;

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
int8_t currentHighestNote = 0;
int8_t lastHighestNote = 0;
int8_t currentLowestNote = 0;
int8_t lastLowestNote = 0;
int8_t currentNote = 0;
int8_t lastCurrentNote = 0;

// Shift Register Mode
bool shiftRegisterMode = false;

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

// Encoder long press timing
const float ENCODER_LONG_PRESS_MS = 1500.0f;
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

// Parameter objects for trigger sequence controls
Parameter densityParam, noteParam, enableParam, resetParam;

struct panelStruct
{
    std::string     name;
    std::string     input1Name;
    std::string     input2Name;
    std::string     input3Name;
    std::string     input4Name;
    float           values[4];
};
panelStruct displayPanels[5] = {
    { 
        name: "ADSR", 
        input1Name: "A", 
        input2Name: "D/R", 
        input3Name: "S",
        input4Name: "Min",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        name: "Panning",
        input1Name: "Freq",
        input2Name: "Amp",
        input3Name: "",
        input4Name: "",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        name: "Oscillators",
        input1Name: "Waveform",
        input2Name: "",
        input3Name: "",
        input4Name: "",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        name: "TrigSeq",
        input1Name: "Density",
        input2Name: "Note",
        input3Name: "Enable",
        input4Name: "Reset",
        values: {0.0f, 0.0f, 0.0f, 0.0f}
    },
    {
        name: "Tuning",
        input1Name: "Tuning",
        input2Name: "Range",
        input3Name: "MIDI",
        input4Name: "Osc",
        values: {0.0f, 0.0f, 1.0f, 1.0f}
    }
};
int panelModesCount = sizeof(displayPanels) / sizeof(displayPanels[0]);
panelStruct currentPanel;
int noteCount = 0;

float previousKnobState [4];

struct voiceStruct
{
    int8_t note;
    int8_t velocity;
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

void      initOscillators(float samplerate);
void      UpdateOscillators();

bool      knobChanged = false;

void DisplayMessage(const char* str)
{
    // hw.display.Fill(false);
    hw.display.SetCursor(0, 50);
    hw.display.WriteString(str, Font_6x8, true);
    hw.display.Update();
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
    float angle = 0.25 * M_PI + pan * 0.5f * M_PI;
    *left       = 0.5 * value * abs(cosf(angle));
    *right      = 0.5 * value * abs(sinf(angle));
}

void ApplyPanning(float* data) {
    float L1, R1, L2, R2, L3, R3, L4, R4;
    panOutput = pan.Process();
    // float dryl, dryr, sendLeft, sendRight, wetl, wetr; // Effects Vars
    //     dryl  = results[0] * 0.5 + results[2] * 0.5;
    //     dryr = results[1] * 0.5 + results[3] * 0.5;
    //     sendLeft = dryl * 0.8;
    //     sendRight = dryr * 0.8;
    PanEqualPowerStereo(panOutput, data[0], &L1, &R1);
    pan.PhaseAdd(0.25f);
    PanEqualPowerStereo(pan.Process(), data[1], &L2, &R2);
    pan.PhaseAdd(0.25f);
    PanEqualPowerStereo(pan.Process(), data[2], &L3, &R3);
    pan.PhaseAdd(0.25f);
    PanEqualPowerStereo(pan.Process(), data[3], &L4, &R4);
    pan.PhaseAdd(0.25f);

    // for (size_t i = 0; i < 4; i++)
    // {

        // pan.SetFreq(hw.GetKnobValue(DaisyPatch::CTRL_1) * 1000.0f);
        // pan.SetAmp(hw.GetKnobValue(DaisyPatch::CTRL_2));
        // pan.SetWaveform(Oscillator::WAVE_SIN);
        // panOutput = pan.Process();
        // data[i] = data[i] * panOutput;
    // }
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
        float velocityFactor = (voices[i].velocity > 0) ? voices[i].velocity / 127.0f : 0.0f;
        envVal = std::max(envelopes[i].envSig * velocityFactor, voicesMinLevel);
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

    // UpdateOscillators();
    

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
         
        // UpdateOscillators();       

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
        
        // Set initial ADSR values - these will be updated by ProcessKnobs based on current panel
        // for some reason this is currently not working
        envelopes[i].env.SetTime(ADSR_SEG_ATTACK, 0.0001f);    // 1ms attack
        envelopes[i].env.SetTime(ADSR_SEG_DECAY, 0.5f);        // 500ms decay
        envelopes[i].env.SetTime(ADSR_SEG_RELEASE, 0.5f);       // 200ms release
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
    // Only passthrough MIDI when not in shift register mode
    if (!shiftRegisterMode) {
        PassthroughMidiMessage(m);
    }

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
            } else {
                // Original behavior
                envelopes[p.channel - channelOffset].gate = true;
                voices[p.channel - channelOffset].note = p.note;
                voices[p.channel - channelOffset].velocity = p.velocity;
                envelopes[p.channel - channelOffset].env.Retrigger(true);
                
                // Update internal oscillator frequencies for all voices
                if (useInternalOscillators) {
                    float freq = MidiNoteToFrequency(p.note, p.channel);
                    int voiceIndex = p.channel - channelOffset;
                    if (voiceIndex >= 0 && voiceIndex < 4) {
                        voiceInterpOsc[voiceIndex].SetFreq(freq);
                    }
                }
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
            // note selection is handled by sending CC signal, scaled to 0-63
            SendMidiMesssage(p.channel * 16 + 2, 15, "CC");
            // also send 
            // SendMidiMesssage(1, 13, "TRIGGER_ON");
            
            // Set timer for trigger off after 10ms delay
            triggerOffTime = hw.seed.system.GetNow() + TRIGGER_OFF_DELAY_MS;
            triggerOffPending = true;

            // probably move outside of audio callback
            char message[60];
            snprintf(message, 60, "Note:%d Ch:%d", p.note, static_cast<int>(p.channel));
            DisplayMessage(message);
            
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
            } else {
                // Original behavior
                envelopes[p.channel - channelOffset].gate = false;
                
                // Clear voice data when note is released
                voices[p.channel - channelOffset].note = 0;
                voices[p.channel - channelOffset].velocity = 0;
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
        displayPanels[0].values[i] = hw.controls[i].Process();
        displayPanels[1].values[i] = hw.controls[i].Process();
        displayPanels[2].values[i] = hw.controls[i].Process();
        displayPanels[3].values[i] = hw.controls[i].Process();
        displayPanels[4].values[i] = hw.controls[i].Process();
    }
    
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

    // Initialize parameter objects for trigger sequence controls
    densityParam.Init(hw.controls[0], 0.0f, 16.5f, Parameter::LINEAR); // maybe does not actually reach the maximum value
    noteParam.Init(hw.controls[1], 36.0f, 84.0f, Parameter::LINEAR);
    enableParam.Init(hw.controls[2], 0.0f, 1.0f, Parameter::LINEAR);
    resetParam.Init(hw.controls[3], 0.0f, 1.0f, Parameter::LINEAR);

    UpdateOled();

    // start MIDI handler
    hw.midi.StartReceive();

    for (int i = 0; i < 4; i++)
    {
        voices[i].note = 60;

        // pluck init
        // plucks[i].decay = 1.0;
        // plucks[i].wetDry = 0.5;
        // plucks[i].synth.SetDecay(1.0);
        // plucks[i].synth.Init(samplerate);

    }
    // synth.Init(samplerate);

    // 
    InitPan(samplerate);
    
    // Initialize internal oscillators for voices 1 and 3
    // voice1Osc.Init(samplerate);
    // voice1Osc.SetFreq(440.0f);  // Default frequency
    // voice1Osc.SetAmp(1.0f);
    // voice1Osc.SetWaveform(Oscillator::WAVE_SIN);
    
    // voice3Osc.Init(samplerate);
    // voice3Osc.SetFreq(440.0f);  // Default frequency
    // voice3Osc.SetAmp(1.0f);
    // voice3Osc.SetWaveform(Oscillator::WAVE_SIN);
    
    // Initialize interpolated oscillators for all 4 voices
    for (int i = 0; i < 4; i++) {
        voiceInterpOsc[i].Init(samplerate);
        voiceInterpOsc[i].SetFreq(440.0f);
        voiceInterpOsc[i].SetAmp(1.0f);
        voiceInterpOsc[i].SetWaveformParam(0.0f);  // Start with sine wave
    }
    
    // initOscillators(samplerate);

    // Initialize parameters with linear scaling
    Parameter densityParam, noteParam, enableParam, resetParam;
    densityParam.Init(hw.controls[0], 0.0f, 16.0f, Parameter::LINEAR);
    noteParam.Init(hw.controls[1], 36.0f, 84.0f, Parameter::LINEAR);
    enableParam.Init(hw.controls[2], 0.0f, 1.0f, Parameter::LINEAR);
    resetParam.Init(hw.controls[3], 0.0f, 1.0f, Parameter::LINEAR);

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
            SendMidiMesssage(currentNote, 13, "NOTE_OFF"); // might be a bug here in currentNote changing while trigger off is pending
            // SendMidiMesssage(0, 13, "TRIGGER_OFF");
            triggerOffPending = false;
        }
        
        // Check for sequence trigger off timing
        if (sequenceTriggerOffPending && currentTime >= sequenceTriggerOffTime)
        {
            SendMidiMesssage(triggerNote, 9, "TRIGGER_OFF");
            sequenceTriggerOffPending = false;
        }

        // Check for MIDI clock timing
        if (clockEnabled && (currentTime - lastClockTime) >= clockInterval)
        {
            SendMidiClock();
            lastClockTime = currentTime;
        }

        // Update trigger sequence
        UpdateTriggerSequence();

        // envelopes[p.channel].trig = true;

        // Prepare buffers for sampler as needed
        // sampler.Prepare();

        // hw.DelayMs(1);
    }
}

void UpdateOled()
{
    // hw.display.Fill(false);

    hw.display.SetCursor(0, 0);
    std::string str  = currentPanel.input1Name;
    char*      cstr = &str[0];
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(35, 0);
    str = currentPanel.input2Name;
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(70, 0);
    str = currentPanel.input3Name;
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(105, 0);
    str = currentPanel.input4Name;
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(0, 20);

    str = currentPanel.name;
    hw.display.WriteString(cstr, Font_7x10, true);
    
    // Show shift register mode indicator and BPM
    if (shiftRegisterMode) {
        hw.display.SetCursor(0, 35);
        hw.display.WriteString("SHIFT", Font_6x8, true);
    } else {
        // erase shift text
        hw.display.SetCursor(0, 35);
        hw.display.WriteString("     ", Font_6x8, true);
    }
    
    // Show current BPM
    hw.display.SetCursor(80, 50);
    // Use a fixed-width format that always takes the same space
    hw.display.SetCursor(80, 50);
    char bpmStr[40];
    snprintf(bpmStr, sizeof(bpmStr), "BPM:%3d", clockBpm); // Always 7 chars
    hw.display.WriteString(bpmStr, Font_6x8, true);
    
    // Show trigger sequence pattern when in TrigSeq mode
    if (currentPanel.name == "TrigSeq") {
        hw.display.SetCursor(0, 50);
        char seqStr[20];
        snprintf(seqStr, sizeof(seqStr), "Step:%02d", currentSequenceStep);
        hw.display.WriteString(seqStr, Font_6x8, true);

        // Show density value
        int numTriggers = 0;
        for (int i = 0; i < TRIGGER_SEQUENCE_LENGTH; i++) {
            if (triggerSequence[i]) numTriggers++;
        }
        hw.display.SetCursor(50, 50);
        char densityStr[20];
        snprintf(densityStr, sizeof(densityStr), "D:%02d", numTriggers);
        hw.display.WriteString(densityStr, Font_6x8, true);
        
        // Show sequence pattern as dots
        hw.display.SetCursor(0, 40);
        for (int i = 0; i < TRIGGER_SEQUENCE_LENGTH; i++) {
            if (triggerSequence[i]) {
                hw.display.WriteString("*", Font_6x8, true);
            } else {
                hw.display.WriteString("-", Font_6x8, true);
            }
        }
    }
    // Show tuning information when in Tuning mode
    else if (currentPanel.name == "Tuning") {
        // Show current tuning name
        const ScalaTuning* tuning = GetTuningByIndex(currentTuningIndex);
        hw.display.SetCursor(0, 35);
        hw.display.WriteString(tuning->name, Font_6x8, true);
        
        // Show pitch bend range
        hw.display.SetCursor(0, 50);
        char rangeStr[20];
        snprintf(rangeStr, sizeof(rangeStr), "Range:%3.0fc", pitchBendRange);
        hw.display.WriteString(rangeStr, Font_6x8, true);
        
        // Show MIDI and Osc status
        hw.display.SetCursor(70, 50);
        char statusStr[20];
        snprintf(statusStr, sizeof(statusStr), "%s%s", 
                sendPitchBendMidi ? "M" : "-", 
                applyToInternalOsc ? "O" : "-");
        hw.display.WriteString(statusStr, Font_6x8, true);
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
    
    // Detect long press: trigger when held >= 3 seconds
    if(encoderPressed && timeHeld >= ENCODER_LONG_PRESS_MS && !longPressHandled)
    {
        // Long press detected - toggle shift register mode
        shiftRegisterMode = !shiftRegisterMode;
        longPressHandled = true;
        
        // Clear all state when switching modes to prevent MIDI artifacts
        // Reset shift register (this sends note-offs for active voices)
        shift_register.Reset();
        ApplyShiftRegisterState();
        
        // Clear all voice and envelope states
        for(size_t i = 0; i < 4; ++i)
        {
            voices[i].note = 0;
            voices[i].velocity = 0;
            envelopes[i].gate = false;
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
    float knobThreshold = 0.0003; // emperically derived, lower values produce jitter

    for (int i = 0; i < 4; i++)
    {
        inputs[i] = hw.controls[i].Process();
        if (fabs(inputs[i] - previousKnobState[i]) > knobThreshold)
        {
            inputIndex = i;
            if (inputs[i] > 0.1f) {
                currentPanel.values[i] = inputs[i];
                knobChanged = true;
            }
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
                    // Attack: logarithmic scaling 0.001s to 1.0s
                    {
                        float attackTime = 0.001f * powf(1000.0f, inputs[0]);
                        envelopes[i].env.SetTime(ADSR_SEG_ATTACK, attackTime);
                    }
                    break;
                case 1:
                    // Decay/Release: logarithmic scaling 0.001s to 5.0s
                    {
                        float decayTime = 0.001f * powf(5000.0f, inputs[1]);
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
    else if (currentPanel.name == "Panning")
    {
        switch(inputIndex)
        {
            case 0:
                pan.SetFreq(inputs[0] * 1.0f);
                break;
            case 1:
                pan.SetAmp(inputs[1]);
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
    else if (currentPanel.name == "Oscillators")
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
    else if (currentPanel.name == "TrigSeq")
    {
        // Process parameters continuously
        triggerNote = static_cast<uint8_t>(noteParam.Process());
        sequenceEnabled = (enableParam.Process() > 0.5f);
        
        // Reset sequence (when knob is turned to max and back)
        if (resetParam.Process() > 0.9f) {
            currentSequenceStep = 0;
            lastSequenceStepTime = hw.seed.system.GetNow();
        }
        
        // Only regenerate pattern when density knob changes
        if (inputIndex == 0) {
            // Round the parameter value to ensure we get exact integer values
            int numTriggers = static_cast<int>(densityParam.Process() + 0.5f);
            numTriggers = std::max(0, std::min(numTriggers, static_cast<int>(TRIGGER_SEQUENCE_LENGTH)));
            
            // Generate Euclidean rhythm pattern
            GenerateEuclideanRhythm(numTriggers, TRIGGER_SEQUENCE_LENGTH, triggerSequence);
            knobChanged = true;
        }
    }
    else if (currentPanel.name == "Tuning")
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
                    }
                }
                break;
            default:
                break;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        previousKnobState[i] = inputs[i];
    }
}



void ProcessControls()
{
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    ProcessEncoder();
    ProcessKnobs();
    // ProcessGates(); // not using this for now in 4-voice mode
}

void InitPan(float samplerate)
{
    pan.Init(samplerate);
    pan.SetFreq(0.03f);
    pan.SetAmp(1);
    pan.SetWaveform(Oscillator::WAVE_SIN);
}

// ---------------------------------------------------------------
// more like a scratchpad below

void initOscillators(float samplerate) {
    osc1.Init(samplerate);
    osc2.Init(samplerate);
    lfo1.Init(samplerate);
    lfo2.Init(samplerate);
    lfo3.Init(samplerate);

    lfo1.SetFreq(5.0f);
    lfo1.SetAmp(1);
    lfo1.SetWaveform(Oscillator::WAVE_SIN);

    lfo2.SetFreq(5.0f);
    lfo2.SetAmp(1);
    lfo2.SetWaveform(Oscillator::WAVE_SIN);

    lfo3.SetFreq(1.5f);
    lfo3.SetAmp(1);
    lfo3.SetWaveform(Oscillator::WAVE_SIN);
}

void ProcessGates()
{
    for(int i = 0; i < 4; i++)
    {
        if(hw.gate_input[i].Trig())
        {
            // envelopes[i].env.Retrigger(true);
        }
    }
}

void UpdateOscillators() {
    float lfo1out = lfo1.Process();
    float lfo2out = lfo2.Process();
    float lfo3out = lfo3.Process();
    osc1.SetIndex(0);
    osc1.SetRatio(lfo2out + 3);
    osc2.SetIndex(0);
    osc2.SetRatio(lfo2out + 3);

    // testing LFO out
    hw.seed.dac.WriteValue(DacHandle::Channel::ONE,
                    (lfo1out) * 4095);
    hw.seed.dac.WriteValue(DacHandle::Channel::TWO,
                    (lfo2out) * 4095);

    // FM synthesis with internal oscillators doesn't seem to track well with MIDI conversion on MIDI 1U, leaving out for now
    // revisit this when implementing note to frequency conversion
    // float output;
    // output = osc1.Process() * envelopes[0].envSig;
    // out[1][i] = output;
    // output = osc2.Process() * envelopes[1].envSig;
    // out[3][i] = output;
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
            if(state.needs_retrigger)
            {
                envelopes[i].env.Retrigger(true);
            }
            // Use gate_on state from the library (tracks note-on/off)
            envelopes[i].gate  = state.gate_on;
            voices[i].note     = static_cast<int8_t>(state.note);
            voices[i].velocity = static_cast<int8_t>(state.velocity);
            
            // Update internal oscillator frequencies for all voices
            if (useInternalOscillators && state.gate_on) {
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
        // Send trigger on MIDI channel 10 (channel 9 in 0-based indexing)
        SendMidiMesssage(triggerNote, 9, "TRIGGER_ON");

        // Schedule trigger off after a short duration (50ms)
        sequenceTriggerOffTime = hw.seed.system.GetNow() + 50;
        sequenceTriggerOffPending = true;
    }
    
    // Advance to next step
    currentSequenceStep = (currentSequenceStep + 1) % TRIGGER_SEQUENCE_LENGTH;
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