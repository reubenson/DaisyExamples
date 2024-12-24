#include "daisy_patch.h"
#include "daisysp.h"
#include <string>

using namespace daisy;
using namespace daisysp;

DaisyPatch      hw;
Fm2             osc1, osc2;
Oscillator      pan, lfo1, lfo2, lfo3;
SdmmcHandler    sdcard;
FatFSInterface  fsi;
WavPlayer       sampler;

int panelMode;
float cvOut1;
float cvOut2;
float panOutput;
int8_t currentHighestNote = 0;
int8_t lastHighestNote = 0;

struct panelStruct
{
    std::string     name;
    std::string     input1Name;
    std::string     input2Name;
    std::string     input3Name;
    std::string     input4Name;
    float           values[4];
};
panelStruct displayPanels[2] = {
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
    }
    // ,
    // { 
    //     name: "Reverb", 
    //     input1Name: "Wet/Dry",
    //     input2Name: "Decay",
    //     input3Name: "",
    //     input4Name: ""
    // }
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
    Parameter attackParam;
    Parameter decayParam;
    Parameter curveParam;
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
ReverbSc  verb;

envStruct envelopes[4];
void      ProcessControls();
// void      UpdateEnvelopes();
void      ApplyVCAs();
void      ApplyPanning(float* data);
void      UpdateOled();
void      plucksApply();
void      InitPan(float samplerate);

void      initOscillators(float samplerate);
void      InitSampler();
void      UpdateOscillators();
void      UpdateSampler();

bool      knobChanged = false;

void DisplayMessage(const char* str)
{
    hw.display.Fill(false);
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

// Apply VCA to inputs based on envelope values
void ApplyVCAs(float* data) {
    float envMax = 0.0f;
    float envVal = 0.0f;
    float velOffset = 0.35; // needed to bias the Intellijel vactrol
    for (size_t i = 0; i < 4; i++) {
        // char message[60];
        // snprintf(message, 60, "val: %d", voices[i].note);
        data[i] = data[i] * envelopes[i].envSig;

        envVal = envelopes[i].envSig * (velOffset + (1 - velOffset) * voices[i].velocity / 127.0f);

        if (envVal > envMax)
        {
            // TODO make velocity more responsive, not just on noteOn
            // envMax = envelopes[i].envSig * (velOffset + (1 - velOffset) * voices[i].velocity / 127.0f);
            envMax = envVal;
        }
    }

    cvOut1 = IncrementTowards(cvOut1, envMax);

    // and LFO signal onto cvOut1
    
    hw.seed.dac.WriteValue(DacHandle::Channel::TWO, cvOut1 * 4095);
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    ProcessControls();
    // UpdateEnvelopes();

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

    // UpdateSampler();
    // UpdateOscillators();
    

    float results[4];

    for(size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            // results[j] = const_cast<float*>(&in[j][i]);
            results[j] = in[j][i];
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

        // output wav
        // out[3][i] = s162f(sampler.Stream()) * 1.0f;
        

        // why does turning this on kill audio when the panel is on Reverb? makes no sense at all
        // verb.Process(sendLeft, sendRight, &wetl, &wetr);

        out[0][i] = results[0];
        out[1][i] = results[1];
    }

    bool shouldUpdateDisplay = knobChanged;
    if (shouldUpdateDisplay)
    {
        UpdateOled();
        knobChanged = false;
    }
}

void InitEnvelopes(float samplerate)
{
    for(int i = 0; i < 4; i++)
    {
        //envelope values and Init
        envelopes[i].env.Init(samplerate);
        // envelopes[i].env.SetMax(1);
        // envelopes[i].env.SetMin(0);
        // envelopes[i].env.SetCurve(0);

        envelopes[i].attackParam.Init(hw.controls[0], .01, 1, Parameter::LINEAR);
        envelopes[i].decayParam.Init(hw.controls[1], .01, 1, Parameter::LINEAR);
        envelopes[i].curveParam.Init(hw.controls[0], -10, 10, Parameter::LINEAR);
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
    if (type == "NOTE_ON")
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x90 + channel), value, 127};
        hw.midi.SendMessage(bytes, 3);
    }
    else if (type == "NOTE_OFF")
    {
        uint8_t bytes[3] = {static_cast<uint8_t>(0x80 + channel), value, 0};
        hw.midi.SendMessage(bytes, 3);
    }
    // else if (type == 'CC')
    // {
    //     uint8_t bytes[3] = {0xB0, , note};
    //     hw.midi.SendMessage(bytes, 3);
    // }
}

int8_t getCurrentHighestNote() {
    int8_t highestNote = voices[0].note;
    for (int i = 1; i < 4; i++)
    {
        highestNote = std::max(highestNote, voices[i].note);
    }
    return highestNote;
}

void HandleMidiMessage(MidiEvent m)
{   
    // no longer need passthrough after MIDI 1U firmware update
    PassthroughMidiMessage(m);

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
            envelopes[p.channel - channelOffset].gate = true;
            // // need to turn off trig on the next tick, or a few ticks?
            // envelopes[p.channel].trig = true;
            voices[p.channel - channelOffset].note = p.note;
            voices[p.channel - channelOffset].velocity = p.velocity;

            envelopes[p.channel - channelOffset].env.Retrigger(true);

            char message[60];
            snprintf(message, 60, "Note: %d Ch: %d", p.note, static_cast<int>(p.channel));
            DisplayMessage(message);

            // pass highest currently held note to Intellijel via channel 16 and CC 1
            currentHighestNote = getCurrentHighestNote();
            if (currentHighestNote != lastHighestNote) {
                SendMidiMesssage(currentHighestNote, 15, "NOTE_ON");
            }
            // turn off previous note
            if (lastHighestNote != 0 && lastHighestNote != currentHighestNote) {
                SendMidiMesssage(lastHighestNote, 15, "NOTE_OFF");
            }
            lastHighestNote = currentHighestNote;
            
            noteCount++;
        }
        break;
        case NoteOff:
        {
            NoteOffEvent p = m.AsNoteOff();
            envelopes[p.channel - channelOffset].gate = false;
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

    panelMode = 0;
    currentPanel = displayPanels[panelMode];
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

        // reverb init
        // verb.Init(samplerate);
        // verb.SetFeedback(0.85f);
        // verb.SetLpFreq(2000.0f);
    }
    // synth.Init(samplerate);

    // 
    InitPan(samplerate);
    // initOscillators(samplerate);
    // InitSampler();

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

        
        // envelopes[p.channel].trig = true;

        // Prepare buffers for sampler as needed
        // sampler.Prepare();

        // hw.DelayMs(1);
    }
}

void UpdateOled()
{
    hw.display.Fill(false);

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

    hw.display.SetCursor(0, 50);

    str = currentPanel.name;
    hw.display.WriteString(cstr, Font_6x8, true);
    
    // draw current knob values
    for (int i = 0; i < 4; i++)
    {
        // hw.display.SetCursor(0 + (i * 20), 25);
        float val = currentPanel.values[i];

        // bug: val oscillates between 0 and the actual value???
        // currently this seems to only get called when it is incorrectly reading 0 ... ?
        // if (val > 0.01f){
            char printme[50];
            snprintf(printme, sizeof(printme), "val: %d", static_cast<int>(val * 200));
            DisplayMessage(printme);
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
    int edge = hw.encoder.RisingEdge();
    panelMode += edge;
    panelMode = panelMode % panelModesCount;
    currentPanel = displayPanels[panelMode];

    if(edge != 0)
    {
        UpdateOled();
    }
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
                        envelopes[i].env.SetTime(ADSR_SEG_ATTACK, inputs[0]);
                        break;
                    case 1:
                        // envelopes[i].env.SetTime(ADSR_SEG_DECAY,
                        //             envelopes[i].decayParam.Process());
                        // envelopes[i].env.SetTime(ADSR_SEG_RELEASE,
                        //             envelopes[i].decayParam.Process());
                        // DisplayMessage(std::to_string(inputIndex).c_str());
                        envelopes[i].env.SetTime(ADSR_SEG_DECAY, inputs[1]);
                        envelopes[i].env.SetTime(ADSR_SEG_RELEASE, inputs[1]);
                        break;
                    case 2:
                        envelopes[i].env.SetSustainLevel(inputs[2]);
                        break;
                    case 3:
                        break;
                    default:
                        break;
                }
                // envelopes[i].env.SetTime(ADSR_SEG_ATTACK,
                //                         hw.GetKnobValue(DaisyPatch::CTRL_1));
                // envelopes[i].env.SetTime(ADSR_SEG_DECAY,
                //                         hw.controls[1].Process());
                // envelopes[i].env.SetTime(ADSR_SEG_RELEASE,
                //                         hw.controls[1].Process());
                // envelopes[i].env.SetSustainLevel(hw.controls[2].Process());                                 
                // envelopes[i].env.SetTime(ADSR_SEG_ATTACK,
                //                      envelopes[i].attackParam.Process());
                // envelopes[i].env.SetTime(ADSR_SEG_DECAY,
                //                         envelopes[i].decayParam.Process());
                // envelopes[i].env.SetTime(ADSR_SEG_RELEASE,
                //                         envelopes[i].decayParam.Process());
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
    else if (currentPanel.name == "Reverb")
    {
        for (size_t j = 0; j < 4; j++)
        {
            // verb.Init(samplerate);
            // verb.SetFeedback(inputs[1]);
            // verb.SetLpFreq(2000.0f);
            // plucks[j].wetDry = inputs[0];
            // plucks[j].decay = inputs[1];
        }
        
        // hw.controls[0].Process();
        // ProcessPluck();
    }
    else if (currentPanel.name == "Pluck")
    {
        // DisplayMessage("inside");
        for (size_t j = 0; j < 4; j++)
        {
            plucks[j].wetDry = inputs[0];
            // plucks[j].decay = inputs[1];
        }
        
        // hw.controls[0].Process();
        // ProcessPluck();
    }

    for (int i = 0; i < 4; i++)
    {
        envelopes[i].envSig = std::max(envelopes[i].env.Process(envelopes[i].gate), inputs[3]);
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
    pan.SetFreq(0.015f);
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

void UpdateSampler()
{
    // handle wav selection
    int32_t inc;
    // Change file with encoder.
    inc = hw.encoder.Increment();
    // DisplayMessage((std::to_string(inc)).c_str());
    if(inc > 0)
    {
        size_t fileCount = sampler.GetNumberFiles();
        // DisplayMessage((std::to_string(fileCount)).c_str());
        size_t curfile;
        curfile = sampler.GetCurrentFile();
        DisplayMessage((std::to_string(curfile + 1)).c_str());
        if(curfile < sampler.GetNumberFiles() - 1)
        {
            sampler.Open(curfile + 1);
            sampler.SetLooping(true);
            sampler.Restart();
        }
    }
    else if(inc < 0)
    {
        size_t curfile;
        curfile = sampler.GetCurrentFile();
        DisplayMessage((std::to_string(curfile - 1)).c_str());
        if(curfile > 0)
        {
            sampler.Open(curfile - 1);
            sampler.SetLooping(true);
            sampler.Restart();
        }
    }
}

void InitSampler() {
    SdmmcHandler::Config sd_cfg;
    sd_cfg.Defaults();
    sd_cfg.speed = SdmmcHandler::Speed::MEDIUM_SLOW;
    sd_cfg.width = SdmmcHandler::BusWidth::BITS_1;
    sdcard.Init(sd_cfg);
    fsi.Init(FatFSInterface::Config::MEDIA_SD);
    f_mount(&fsi.GetSDFileSystem(), "/", 1);

    sampler.Init(fsi.GetSDPath());
    sampler.SetLooping(true);
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