/**
 * AETHER ESP-NOW Gateway - Test Pattern Generator
 * 
 * Generates DMX test patterns when no serial data is being received.
 * Useful for testing nodes without a Pi connected.
 */

#ifndef AETHER_TEST_PATTERN_H
#define AETHER_TEST_PATTERN_H

#include <Arduino.h>
#include "../common/config.h"

// ═══════════════════════════════════════════════════════════════════
// TEST PATTERN GENERATOR
// ═══════════════════════════════════════════════════════════════════

class TestPatternGenerator {
public:
    TestPatternGenerator();
    
    /**
     * Set the pattern type
     * @param pattern Pattern type (TEST_PATTERN_*)
     */
    void setPattern(uint8_t pattern);
    
    /**
     * Get current pattern type
     */
    uint8_t getPattern() const { return pattern_; }
    
    /**
     * Update the pattern (call periodically)
     * @param dmx_data Buffer to write pattern into (512 bytes)
     * @return true if pattern was updated
     */
    bool update(uint8_t* dmx_data);
    
    /**
     * Check if it's time to update
     */
    bool shouldUpdate() const;
    
    /**
     * Get current frame count
     */
    uint32_t getFrameCount() const { return frame_count_; }
    
    /**
     * Reset the pattern to initial state
     */
    void reset();

private:
    uint8_t pattern_;
    uint32_t frame_count_;
    unsigned long last_update_;
    float phase_;
    
    // Pattern generators
    void generateSine(uint8_t* dmx_data);
    void generateChase(uint8_t* dmx_data);
    void generateFull(uint8_t* dmx_data);
    void generateRamp(uint8_t* dmx_data);
};

// Global instance
extern TestPatternGenerator testPattern;

#endif // AETHER_TEST_PATTERN_H
