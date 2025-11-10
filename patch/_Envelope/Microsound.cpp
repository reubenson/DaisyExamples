#include "Microsound.h"
#include <algorithm>

using namespace daisysp;

namespace microsound {

void PulsarSynth::Init(float sample_rate) {
    sample_rate_ = std::max(sample_rate, 1000.0f);  // Ensure minimum valid sample rate
    frequency_ = 440.0f;
    pulsaret_length_ms_ = 20.0f;
    pulse_width_ = 0.5f;
    modulation_ = 0.0f;
    wet_dry_ = 0.3f;
    
    fundamental_phase_ = 0.0f;
    pulsaret_phase_ = 0.0f;
    buffer_read_pos_ = 0.0f;
    
    pulsaret_active_ = false;
    pulsaret_samples_ = (pulsaret_length_ms_ / 1000.0f) * sample_rate_;
    pulsaret_samples_ = std::max(pulsaret_samples_, 10.0f);  // Ensure minimum safe value
    pulserets_per_period_ = 1;  // Now always 1 pulsaret per period (simpler model)
    current_pulsaret_ = 0;
}

void PulsarSynth::SetFrequency(float freq) {
    frequency_ = std::max(20.0f, std::min(freq, 2000.0f));  // Clamp to reasonable range
}

void PulsarSynth::SetPulsaretLength(float length_ms) {
    // Clamp to 2-30ms range for tight, responsive grains
    pulsaret_length_ms_ = std::max(2.0f, std::min(length_ms, 30.0f));
    pulsaret_samples_ = (pulsaret_length_ms_ / 1000.0f) * sample_rate_;
    // Ensure minimum safe value to prevent division by zero
    pulsaret_samples_ = std::max(pulsaret_samples_, 96.0f);  // ~2ms at 48kHz
}

void PulsarSynth::SetPulseWidth(float width) {
    pulse_width_ = std::max(0.0f, std::min(width, 1.0f));
    // Pulse width now controls duty cycle continuously (0.1 to 1.0)
    // At 0, very sparse pulserets; at 1, dense pulserets filling the period
    // This creates a continuous parameter instead of discrete jumps
}

void PulsarSynth::SetModulation(float mod) {
    modulation_ = std::max(0.0f, std::min(mod, 1.0f));
}

void PulsarSynth::SetWetDry(float mix) {
    wet_dry_ = std::max(0.0f, std::min(mix, 1.0f));
}

// Explicit template instantiations for common buffer sizes
template void PulsarSynth::Process<24000>(const daisysp::DelayLine<float, 24000>&, 
                                          const daisysp::DelayLine<float, 24000>&, 
                                          float&, float&);
template void PulsarSynth::Process<30000>(const daisysp::DelayLine<float, 30000>&, 
                                          const daisysp::DelayLine<float, 30000>&, 
                                          float&, float&);

} // namespace microsound

