#pragma once
#ifndef MICROSOUND_H
#define MICROSOUND_H

#include <stdint.h>
#include <cmath>
#include <algorithm>
#include "daisysp.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace microsound {

/** 
 * PulsarSynth - Pulsar synthesis engine that reads from delay buffers
 * 
 * Implements pulsar synthesis by generating trains of windowed grains (pulserets)
 * at a fundamental frequency. Reads audio from delay buffers for granular processing.
 */
class PulsarSynth {
public:
    PulsarSynth() {}
    ~PulsarSynth() {}
    
    /** Initialize the pulsar synthesizer
     *  \param sample_rate Sample rate of audio engine
     */
    void Init(float sample_rate);
    
    /** Process one sample, reading from delay lines
     *  \param delayL Left delay line reference
     *  \param delayR Right delay line reference  
     *  \param outL Reference to output left sample
     *  \param outR Reference to output right sample
     */
    template<size_t max_size>
    void Process(const daisysp::DelayLine<float, max_size>& delayL,
                 const daisysp::DelayLine<float, max_size>& delayR,
                 float& outL, float& outR) {
        // Initialize outputs
        outL = 0.0f;
        outR = 0.0f;
        
        // Update fundamental phase
        float phase_increment = frequency_ / sample_rate_;
        fundamental_phase_ += phase_increment;
        
        // Trigger pulsaret at the start of each period
        if (fundamental_phase_ >= 1.0f) {
            fundamental_phase_ -= 1.0f;
            
            // Only trigger new pulsaret if none is active OR current one is past 50% complete
            // This prevents retriggering during the attack, which causes low amplitude
            if (!pulsaret_active_ || pulsaret_phase_ >= 0.5f) {
                pulsaret_active_ = true;
                pulsaret_phase_ = 0.0f;
            }
            
            // Set buffer read position based on a fixed delay time
            // Modulation provides direct control over read position
            float sample_rate = sample_rate_;
            float base_delay_ms = 35.0f;  // Base delay of 35ms (when Mod = 0) - ultra-low latency
            
            // Modulation controls position: 0 = 35ms back, 1.0 = 60ms back
            float mod_delay_ms = 25.0f * modulation_;
            float delay_time_ms = base_delay_ms + mod_delay_ms;
            
            buffer_read_pos_ = (delay_time_ms / 1000.0f) * sample_rate;
            
            // Clamp to safe range (35-60ms) - 5ms buffer over max grain size (30ms)
            buffer_read_pos_ = std::max(sample_rate * 0.035f, std::min(buffer_read_pos_, sample_rate * 0.06f));
        }
        
        // Process active pulsaret
        if (pulsaret_active_ && pulsaret_samples_ > 1.0f) {
            // Calculate window amplitude
            float window = HannWindow(pulsaret_phase_);
            
            // Apply pulse width as amplitude scaling
            // Use square root for gentler scaling (0.0->0.0, 0.5->0.71, 1.0->1.0)
            float pw_scale = sqrtf(pulse_width_);
            window *= pw_scale;
            
            // Read from delay lines
            // Fixed playback speed (1.0x) for faithful reproduction without pitch shifting
            float playback_speed = 1.0f;
            float read_offset = pulsaret_phase_ * pulsaret_samples_ * playback_speed;
            float read_pos = buffer_read_pos_ + read_offset;
            
            // Wrap read position to stay in buffer (handle both positive and negative overflow)
            while (read_pos >= static_cast<float>(max_size)) {
                read_pos -= static_cast<float>(max_size);
            }
            while (read_pos < 1.0f) {
                read_pos += static_cast<float>(max_size);
            }
            // Final safety clamp
            read_pos = std::max(1.0f, std::min(read_pos, static_cast<float>(max_size - 1)));
            
            // Read samples with Hermite interpolation for higher quality
            float grain_L = delayL.ReadHermite(read_pos);
            float grain_R = delayR.ReadHermite(read_pos);
            
            // Apply window to output (wet/dry mixing happens in ApplyMicrosound)
            outL = grain_L * window;
            outR = grain_R * window;
            
            // Advance pulsaret phase
            // Guard against division by zero or very small values
            if (pulsaret_samples_ > 1.0f) {
                float pulsaret_phase_increment = 1.0f / pulsaret_samples_;
                pulsaret_phase_ += pulsaret_phase_increment;
            } else {
                // If pulsaret_samples_ is invalid, complete the pulsaret immediately
                pulsaret_phase_ = 1.0f;
            }
            
            // Check if pulsaret is complete
            if (pulsaret_phase_ >= 1.0f) {
                pulsaret_active_ = false;
                pulsaret_phase_ = 0.0f;
            }
        }
    }
    
    /** Set fundamental frequency (from MIDI note)
     *  \param freq Frequency in Hz
     */
    void SetFrequency(float freq);
    
    /** Set pulsaret length (grain size)
     *  \param length_ms Length in milliseconds (1-100ms)
     */
    void SetPulsaretLength(float length_ms);
    
    /** Set pulse width (duty cycle - ratio of period filled with pulserets)
     *  \param width Normalized 0.0-1.0 (0.0 = sparse, 1.0 = dense)
     */
    void SetPulseWidth(float width);
    
    /** Set modulation amount (texture/variation parameter)
     *  \param mod Normalized 0.0-1.0
     */
    void SetModulation(float mod);
    
    /** Set wet/dry mix level
     *  \param mix Normalized 0.0-1.0 (0.0 = dry, 1.0 = wet)
     */
    void SetWetDry(float mix);

private:
    /** Generate Hann window value for given phase
     *  \param phase Normalized phase 0.0-1.0
     *  \return Window amplitude 0.0-1.0
     */
    inline float HannWindow(float phase) {
        if (phase < 0.0f || phase > 1.0f) return 0.0f;
        return 0.5f * (1.0f - cosf(2.0f * M_PI * phase));
    }
    
    float sample_rate_;
    float frequency_;           // Fundamental frequency (Hz)
    float pulsaret_length_ms_;  // Pulsaret length in milliseconds
    float pulse_width_;         // Duty cycle (0.0-1.0)
    float modulation_;          // Modulation amount (0.0-1.0)
    float wet_dry_;             // Wet/dry mix (0.0-1.0)
    
    // Phase accumulators
    float fundamental_phase_;   // Phase for fundamental frequency (0.0-1.0)
    float pulsaret_phase_;      // Phase within current pulsaret (0.0-1.0)
    
    // Buffer read state
    float buffer_read_pos_;     // Current position in delay buffer
    
    // Pulsaret state
    bool pulsaret_active_;      // Whether a pulsaret is currently being generated
    float pulsaret_samples_;    // Length of pulsaret in samples
    int pulserets_per_period_;  // Number of pulserets per period (from pulse_width)
    int current_pulsaret_;      // Current pulsaret index in period
};

} // namespace microsound

#endif // MICROSOUND_H

