/**
 * AETHER ESP-NOW Node - DMX Output
 * 
 * High-priority DMX512 output using esp_dmx library.
 * Runs on dedicated core for consistent timing.
 */

#ifndef AETHER_DMX_OUTPUT_H
#define AETHER_DMX_OUTPUT_H

#include <Arduino.h>
#include <esp_dmx.h>
#include "../common/config.h"

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

struct DmxOutputStats {
    uint32_t frames_sent;           // Total DMX frames output
    uint32_t buffer_swaps;          // Number of buffer updates
    unsigned long last_frame_time;  // millis() of last DMX output
    unsigned long last_swap_time;   // millis() of last buffer swap
    float actual_fps;               // Calculated output FPS
    
    void reset() {
        frames_sent = 0;
        buffer_swaps = 0;
        last_frame_time = 0;
        last_swap_time = 0;
        actual_fps = 0;
    }
};

// ═══════════════════════════════════════════════════════════════════
// DMX OUTPUT CLASS
// ═══════════════════════════════════════════════════════════════════

class DmxOutput {
public:
    DmxOutput();
    
    /**
     * Initialize DMX output
     * @return true if successful
     */
    bool begin();
    
    /**
     * Update the pending DMX buffer
     * This doesn't immediately affect output; swapBuffers() does that.
     * @param dmx_data 512 bytes of DMX channel data
     */
    void updateBuffer(const uint8_t* dmx_data);
    
    /**
     * Swap pending buffer to active (thread-safe)
     * Call this after updateBuffer() to apply changes.
     */
    void swapBuffers();
    
    /**
     * Send a DMX frame (call from main loop or timer)
     * Uses the active buffer.
     */
    void sendFrame();
    
    /**
     * Start automatic DMX output at specified rate
     * Uses FreeRTOS task on Core 1.
     * @param hz Output rate in Hz (typically 40)
     */
    void startAutoOutput(uint8_t hz = DMX_REFRESH_HZ);
    
    /**
     * Stop automatic output
     */
    void stopAutoOutput();
    
    /**
     * Check if DMX is outputting
     */
    bool isRunning() const { return running_; }
    
    /**
     * Get a specific channel value from active buffer
     */
    uint8_t getChannel(uint16_t channel) const;
    
    /**
     * Get statistics
     */
    const DmxOutputStats& getStats() const { return stats_; }
    
    /**
     * Reset statistics
     */
    void resetStats() { stats_.reset(); }

private:
    bool initialized_;
    bool running_;
    DmxOutputStats stats_;
    
    // Double buffer for thread safety
    uint8_t buffer_a_[DMX_PACKET_SIZE];  // Includes start code at [0]
    uint8_t buffer_b_[DMX_PACKET_SIZE];
    uint8_t* active_buffer_;
    uint8_t* pending_buffer_;
    volatile bool swap_pending_;
    
    // FreeRTOS task
    TaskHandle_t dmx_task_;
    uint8_t output_hz_;
    
    // Mutex for buffer swap
    portMUX_TYPE buffer_mutex_;
    
    // FPS calculation
    unsigned long fps_frame_count_;
    unsigned long fps_start_time_;
    
    // Static task function
    static void dmxTaskFunction(void* param);
    void dmxTaskLoop();
};

// Global instance
extern DmxOutput dmxOutput;

#endif // AETHER_DMX_OUTPUT_H
