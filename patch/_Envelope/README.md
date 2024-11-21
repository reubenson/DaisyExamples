# _Envelope

## Author

Reuben Son (modified from other examples)

## Description

VCA with internal triggerable envelopes. Ins 1 and 2 are tied to env1, and Ins 3 and 4 are tied to env 2.
Each input has its own output. The raw envelope signals are also available on the CV outs.
The envelope curves and attack/decay times ae controllable as well.

[Source Code](https://github.com/electro-smith/DaisyExamples/tree/master/patch/QuadEnvelope)

## Controls

| Control | Description | Comment |
| --- | --- | --- |
| Gate Ins | Trigger the internal envelopes | Gate In 1 goes with env1, and Gate In 2 with env2 |
| Encoder Press | Switch between control modes | Controls attack/decay or curve |
| Ctrl1 | Env one attack / curve | Controls envelope one's attack time, or curve, depending on the mode |
| Ctrl2 | Env one decay / nothing | Controls envelope one's decay time, or nothing, depending on the mode |
| Ctrl1 | Env two attack / curve | Controls envelope two's attack time, or curve, depending on the mode |
| Ctrl2 | Env two decay / nothing | Controls envelope two's decay time, or nothing, depending on the mode |
| Audio Ins | Audio to be VCA'd by the envelopes | Ins 1 and 2 go with env1, ins 2 and 4 with env2 |
| Audio Outs | Audio post VCA | Each output goes with its respective input |
| CV Outs | Envelope CV | Goes with envelopes one and two respectively |

## Notes
I want to use the Daisy Patch as the main orchestrator for the synthesizer, interfacing with an external MIDI keyboard. As such the expected inputs and outputs are the following:

MIDI IN
PITCH OUT for CV control of two oscillators
VELOCITY OUT for CV control of filters
GATE OUT for CV control of LPG

^ does not work because there are only two CV outputs.
Instead, I can continue to use the Intellijel 1U MIDI to handle MIDI data for velocity and gate, and use the MIDI passthrough to have DaisyPatch process MIDI note information (for things like glide, alternate tunings, and other modulation from the keyboard). Though, actually the current CV outs are being used passing envelopes, so to pass pitch CV instead I'd have to use the LPG differently