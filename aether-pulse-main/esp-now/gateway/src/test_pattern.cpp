/**
 * AETHER ESP-NOW Gateway - Test Pattern Generator Implementation
 */

#include "test_pattern.h"
#include <math.h>

// Global instance
TestPatternGenerator testPattern;

TestPatternGenerator::TestPatternGenerator()
    : pattern_(TEST_PATTERN_MODE)
    , frame_count_(0)
    , last_update_(0)
    , phase_(0.0f) {
}

void TestPatternGenerator::setPattern(uint8_t pattern) {
    pattern_ = pattern;
    reset();
}

void TestPatternGenerator::reset() {
    frame_count_ = 0;
    phase_ = 0.0f;
}

bool TestPatternGenerator::shouldUpdate() const {
    return (millis() - last_update_) >= TEST_PATTERN_INTERVAL_MS;
}

bool TestPatternGenerator::update(uint8_t* dmx_data) {
    if (!shouldUpdate()) {
        return false;
    }
    
    last_update_ = millis();
    
    // Clear the buffer first
    memset(dmx_data, 0, DMX_UNIVERSE_SIZE);
    
    // Generate the selected pattern
    switch (pattern_) {
        case TEST_PATTERN_SINE:
            generateSine(dmx_data);
            break;
            
        case TEST_PATTERN_CHASE:
            generateChase(dmx_data);
            break;
            
        case TEST_PATTERN_FULL:
            generateFull(dmx_data);
            break;
            
        case TEST_PATTERN_RAMP:
            generateRamp(dmx_data);
            break;
            
        default:
            generateSine(dmx_data);
            break;
    }
    
    frame_count_++;
    
    // Advance phase for next frame
    phase_ += 0.05f;
    if (phase_ >= 2.0f * PI) {
        phase_ -= 2.0f * PI;
    }
    
    return true;
}

void TestPatternGenerator::generateSine(uint8_t* dmx_data) {
    // Sine wave on channels 1-16, each offset by phase
    // Creates a nice flowing effect
    for (int i = 0; i < 16; i++) {
        float channel_phase = phase_ + (i * 0.4f);
        float value = (sinf(channel_phase) + 1.0f) * 0.5f;  // 0.0 to 1.0
        dmx_data[i] = (uint8_t)(value * 255.0f);
    }
    
    // Also put a master dimmer on channel 512 for easy verification
    dmx_data[511] = (uint8_t)((sinf(phase_ * 0.5f) + 1.0f) * 127.5f);
}

void TestPatternGenerator::generateChase(uint8_t* dmx_data) {
    // Chase pattern: one channel at a time moves through 1-16
    int active_channel = (frame_count_ / 4) % 16;  // Change every 4 frames
    
    // Set active channel to full
    dmx_data[active_channel] = 255;
    
    // Set neighbors to partial (creates smoother chase)
    int prev = (active_channel + 15) % 16;
    int next = (active_channel + 1) % 16;
    dmx_data[prev] = 64;
    dmx_data[next] = 64;
    
    // Dimmer indicator on channel 512
    dmx_data[511] = 128;
}

void TestPatternGenerator::generateFull(uint8_t* dmx_data) {
    // All channels at full
    memset(dmx_data, 255, DMX_UNIVERSE_SIZE);
}

void TestPatternGenerator::generateRamp(uint8_t* dmx_data) {
    // Linear ramp across all channels
    // Creates gradient from 0 to 255 across channels 1-256
    // Then 255 to 0 across channels 257-512
    for (int i = 0; i < 256; i++) {
        dmx_data[i] = i;
    }
    for (int i = 256; i < 512; i++) {
        dmx_data[i] = 511 - i;
    }
    
    // Add animation: shift the ramp based on phase
    uint8_t offset = (uint8_t)(phase_ * 40.0f);
    for (int i = 0; i < 16; i++) {
        int idx = (i + offset) % 256;
        dmx_data[i] = idx;
    }
}
