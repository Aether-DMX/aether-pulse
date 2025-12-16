/**
 * AETHER ESP-NOW Node - Frame Buffer Manager
 * 
 * Manages the hold-last-look behavior and optional fade-to-black
 * when signal is lost.
 */

#ifndef AETHER_FRAME_BUFFER_H
#define AETHER_FRAME_BUFFER_H

#include <Arduino.h>
#include "../common/config.h"

// ═══════════════════════════════════════════════════════════════════
// FRAME BUFFER MANAGER
// ═══════════════════════════════════════════════════════════════════

class FrameBufferManager {
public:
    FrameBufferManager();
    
    /**
     * Update with new frame data
     * @param dmx_data 512 bytes of DMX data
     * @param universe Universe number
     * @param seq Sequence number
     */
    void updateFrame(const uint8_t* dmx_data, uint16_t universe, uint32_t seq);
    
    /**
     * Get current output frame (with hold-last-look/fade applied)
     * @param output Buffer to write output to (512 bytes)
     */
    void getOutputFrame(uint8_t* output);
    
    /**
     * Process time-based effects (call from loop)
     * Handles fade-to-black when signal is lost.
     */
    void process();
    
    /**
     * Check if we have valid data
     */
    bool hasValidData() const { return has_data_; }
    
    /**
     * Check if signal is currently active
     */
    bool isSignalActive() const;
    
    /**
     * Get time since last frame
     */
    uint32_t timeSinceLastFrame() const;
    
    /**
     * Get last received sequence number
     */
    uint32_t getLastSeq() const { return last_seq_; }
    
    /**
     * Get last received universe
     */
    uint16_t getLastUniverse() const { return last_universe_; }

private:
    uint8_t frame_data_[DMX_UNIVERSE_SIZE];
    uint8_t fade_data_[DMX_UNIVERSE_SIZE];  // For fade-to-black
    
    bool has_data_;
    bool fading_out_;
    uint16_t last_universe_;
    uint32_t last_seq_;
    unsigned long last_frame_time_;
    unsigned long fade_start_time_;
    
    void startFadeOut();
    void processFadeOut();
};

// Global instance
extern FrameBufferManager frameBuffer;

#endif // AETHER_FRAME_BUFFER_H
