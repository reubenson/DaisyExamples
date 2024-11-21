#include "daisy_patch.h"
#include "daisysp.h"
#include <string>

using namespace daisy;
using namespace daisysp;

DaisyPatch hw;

int curveTimeMode;
bool MidiGate1 = false;
bool MidiGate2 = false;

struct envStruct
{
    Adsr      env;
    Parameter attackParam;
    Parameter decayParam;
    Parameter curveParam;
    float     envSig;
};

envStruct envelopes[2];
void      ProcessControls();

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    //Process control inputs
    ProcessControls();

    for(size_t i = 0; i < size; i++)
    {
        //Get the next envelope samples
        envelopes[0].envSig = std::max(envelopes[0].env.Process(hw.gate_input[0].State()), hw.controls[3].Process());
        envelopes[1].envSig = std::max(envelopes[1].env.Process(hw.gate_input[1].State()), hw.controls[3].Process());
        // envelopes[1].envSig = envelopes[1].env.Process();

        for(size_t chn = 0; chn < 2; chn++)
        {
            //The envelopes effect the outputs in pairs
            out[chn * 2][i]     = in[chn * 2][i] * envelopes[chn].envSig;
            out[chn * 2 + 1][i] = in[chn * 2 + 1][i] * envelopes[chn].envSig;
        }
    }
}

void InitEnvelopes(float samplerate)
{
    for(int i = 0; i < 2; i++)
    {
        //envelope values and Init
        envelopes[i].env.Init(samplerate);
        // envelopes[i].env.SetMax(1);
        // envelopes[i].env.SetMin(0);
        // envelopes[i].env.SetCurve(0);
    }

    //envelope parameters (control inputs)
    envelopes[0].attackParam.Init(
        hw.controls[0], .01, 2, Parameter::EXPONENTIAL);
    envelopes[0].decayParam.Init(
        hw.controls[1], .01, 2, Parameter::EXPONENTIAL);
    envelopes[0].curveParam.Init(hw.controls[0], -10, 10, Parameter::LINEAR);

    envelopes[1].attackParam.Init(
        hw.controls[0], .01, 2, Parameter::EXPONENTIAL);
    envelopes[1].decayParam.Init(
        hw.controls[1], .01, 2, Parameter::EXPONENTIAL);
    envelopes[1].curveParam.Init(hw.controls[0], -10, 10, Parameter::LINEAR);
}

void UpdateOled();

void HandleMidiMessage(MidiEvent m)
{
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();
            if(p.velocity > 0)
            {
                // envelopes[0].env.Retrigger(true);
                // envelopes[1].env.Retrigger(true);
                // MidiGate1 = true;
                // MidiGate2 = true;
            }
        }
        break;
        case NoteOff:
        {
            NoteOffEvent p = m.AsNoteOff();
            // MidiGate1 = false;
            // MidiGate2 = false;
            // envelopes[0].env.Release();
            // envelopes[1].env.Release();
        }
        break;
        default: break;
    }
}

int main(void)
{
    float samplerate;
    hw.Init();
    samplerate = hw.AudioSampleRate();

    InitEnvelopes(samplerate);

    curveTimeMode = 0;

    UpdateOled();

    // start MID handler
    hw.midi.StartReceive();

    // Start the ADC and Audio Peripherals on the Hardware
    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    for(;;)
    {
        // Handle MIDI events
        hw.midi.Listen(); // do I need to keep calling this?
        // Handle MIDI Events
        while(hw.midi.HasEvents())
        {
            HandleMidiMessage(hw.midi.PopEvent());
        }

        //Send the latest envelope values to the CV outs
        hw.seed.dac.WriteValue(DacHandle::Channel::ONE,
                        envelopes[0].envSig * 4095);
        hw.seed.dac.WriteValue(DacHandle::Channel::TWO,
                        envelopes[1].envSig * 4095);
        hw.DelayMs(1);
    }
}

void UpdateOled()
{
    hw.display.Fill(false);

    hw.display.SetCursor(0, 0);
    std::string str  = "A";
    char*       cstr = &str[0];
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(35, 0);
    str = "D/R";
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(70, 0);
    str = "S";
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(105, 0);
    str = "MIN";
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.SetCursor(0, 50);
    //curve or attack/decay mode
    if(curveTimeMode)
    {
        str = "curve";
    }
    else
    {
        str = "ADSR :)";
    }
    hw.display.WriteString(cstr, Font_6x8, true);

    hw.display.Update();
}

void ProcessEncoder()
{
    int edge = hw.encoder.RisingEdge();
    curveTimeMode += edge;
    curveTimeMode = curveTimeMode % 2;

    if(edge != 0)
    {
        UpdateOled();
    }
}

void ProcessKnobs()
{
    for(int i = 0; i < 1; i++)
    {
        if(curveTimeMode == 0)
        {
            envelopes[0].env.SetTime(ADSR_SEG_ATTACK,
                                     envelopes[i].attackParam.Process());
            envelopes[0].env.SetTime(ADSR_SEG_DECAY,
                                     envelopes[i].decayParam.Process());
            envelopes[0].env.SetSustainLevel(hw.controls[2].Process());                                 
            envelopes[0].env.SetTime(ADSR_SEG_RELEASE,
                                     envelopes[i].decayParam.Process());
                                    // 0);
            envelopes[1].env.SetTime(ADSR_SEG_ATTACK,
                                     envelopes[i].attackParam.Process());
            envelopes[1].env.SetTime(ADSR_SEG_DECAY,
                                     envelopes[i].decayParam.Process());
            envelopes[1].env.SetSustainLevel(hw.controls[2].Process());
            envelopes[1].env.SetTime(ADSR_SEG_RELEASE,
                                     envelopes[i].decayParam.Process());
                                    // 0);
        }
        else
        {
            // envelopes[i].env.SetCurve(envelopes[i].curveParam.Process());
            // envelopes[0].env.SetCurve(envelopes[i].curveParam.Process());
            // envelopes[1].env.SetCurve(envelopes[i].curveParam.Process());
        }
    }
}

void ProcessGates()
{
    for(int i = 0; i < 2; i++)
    {
        if(hw.gate_input[i].Trig())
        {
            envelopes[i].env.Retrigger(true);
        }
    }
}

void ProcessControls()
{
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    ProcessEncoder();
    ProcessKnobs();
    ProcessGates();
}
