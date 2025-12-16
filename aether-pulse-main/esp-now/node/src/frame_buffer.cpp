/**
 * AETHER ESP-NOW Node - Frame Buffer Manager Implementation
 */

#include "frame_buffer.h"

// Global instance
FrameBufferManager frameBuffer;

FrameBufferManager::FrameBufferManager()
    : has_data_(false)
    , fading_out_(false)
    , last_universe_(0)
    , last_seq_(0)
    , last_frame_time_(0)
    , fade_start_time_(0) {
    
    memset(frame_data_, 0, DMX_UNIVERSE_SIZE);
    memset(fade_data_, 0, DMX_UNIVERSE_SIZE);
}

void FrameBufferManager::updateFrame(const uint8_t* dmx_data, 
                                      uint16_t universe, uint32_t seq) {
    // Copy new data
    memcpy(frame_data_, dmx_data, DMX_UNIVERSE_SIZE);
    
    // Update metadata
    last_universe_ = universe;
    last_seq_ = seq;
    last_frame_time_ = millis();
    has_data_ = true;
    
    // Cancel any fade-out in progress
    if (fading_out_) {
        fading_out_ = false;
        #if DEBUG_VERBOSE
        Serial.println("[BUF] Signal restored, fade cancelled");
        #endif
    }
}

void FrameBufferManager::getOutputFrame(uint8_t* output) {
    if (!has_data_) {
        // No data yet, output zeros
        memset(output, 0, DMX_UNIVERSE_SIZE);
        return;
    }
    
    if (fading_out_) {
        // Output fade data
        memcpy(output, fade_data_, DMX_UNIVERSE_SIZE);
    } else {
        // Output last good frame (hold-last-look)
        memcpy(output, frame_data_, DMX_UNIVERSE_SIZE);
    }
}

void FrameBufferManager::process() {
    if (!has_data_) return;
    
    // Check for signal timeout
    #if HOLD_LAST_LOOK_TIMEOUT_MS > 0
    if (!fading_out_ && !isSignalActive()) {
        startFadeOut();
    }
    
    if (fading_out_) {
        processFadeOut();
    }
    #endif
}

bool FrameBufferManager::isSignalActive() const {
    if (!has_data_) return false;
    if (last_frame_time_ == 0) return false;
    
    // Consider signal active if we've received data within timeout
    return (millis() - last_frame_time_) < HOLD_LAST_LOOK_TIMEOUT_MS;
}

uint32_t FrameBufferManager::timeSinceLastFrame() const {
    if (last_frame_time_ == 0) return UINT32_MAX;
    return millis() - last_frame_time_;
}

void FrameBufferManager::startFadeOut() {
    #if HOLD_LAST_LOOK_TIMEOUT_MS > 0
    if (fading_out_) return;
    
    // Copy current frame to fade buffer
    memcpy(fade_data_, frame_data_, DMX_UNIVERSE_SIZE);
    fade_start_time_ = millis();
    fading_out_ = true;
    
    Serial.println("[BUF] Signal lost, starting fade to black");
    #endif
}

void FrameBufferManager::processFadeOut() {
    #if HOLD_LAST_LOOK_TIMEOUT_MS > 0 && SIGNAL_LOSS_FADE_MS > 0
    unsigned long elapsed = millis() - fade_start_time_;
    
    if (elapsed >= SIGNAL_LOSS_FADE_MS) {
        // Fade complete, hold at black
        memset(fade_data_, 0, DMX_UNIVERSE_SIZE);
        return;
    }
    
    // Calculate fade multiplier (1.0 -> 0.0)
    float fade = 1.0f - ((float)elapsed / SIGNAL_LOSS_FADE_MS);
    
    // Apply fade to all channels
    for (int i = 0; i < DMX_UNIVERSE_SIZE; i++) {
        fade_data_[i] = (uint8_t)(frame_data_[i] * fade);
    }
    #endif
}
