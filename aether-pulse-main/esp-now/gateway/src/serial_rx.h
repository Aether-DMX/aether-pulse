/**
 * AETHER ESP-NOW Gateway - Serial Frame Receiver
 * 
 * Receives framed DMX data from Raspberry Pi over USB/Serial.
 * Non-blocking state machine with preamble sync and CRC validation.
 */

#ifndef AETHER_SERIAL_RX_H
#define AETHER_SERIAL_RX_H

#include <Arduino.h>
#include "../common/config.h"
#include "../common/packet.h"
#include "../common/crc16.h"

// ═══════════════════════════════════════════════════════════════════
// RECEIVER STATE MACHINE
// ═══════════════════════════════════════════════════════════════════

enum class SerialRxState {
    SYNC_0,         // Waiting for first preamble byte (0xAA)
    SYNC_1,         // Waiting for second preamble byte (0x55)
    HEADER,         // Reading header (universe, seq, length)
    PAYLOAD,        // Reading DMX payload
    CRC_0,          // Reading CRC low byte
    CRC_1           // Reading CRC high byte
};

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

struct SerialRxStats {
    uint32_t frames_received;   // Successfully received frames
    uint32_t crc_errors;        // CRC validation failures
    uint32_t sync_errors;       // Resync events
    uint32_t timeout_errors;    // Timeout resyncs
    uint32_t bytes_received;    // Total bytes processed
    unsigned long last_frame_time; // millis() of last valid frame
    
    void reset() {
        frames_received = 0;
        crc_errors = 0;
        sync_errors = 0;
        timeout_errors = 0;
        bytes_received = 0;
        last_frame_time = 0;
    }
};

// ═══════════════════════════════════════════════════════════════════
// SERIAL RECEIVER CLASS
// ═══════════════════════════════════════════════════════════════════

class SerialReceiver {
public:
    // Callback type for when a complete frame is received
    using FrameCallback = void (*)(uint16_t universe, uint32_t seq, 
                                   const uint8_t* dmx_data, uint16_t length);

    SerialReceiver();
    
    /**
     * Initialize the serial receiver
     * @param serial Hardware serial port to use
     * @param baud_rate Baud rate (default from config)
     */
    void begin(HardwareSerial& serial, uint32_t baud_rate = SERIAL_BAUD_RATE);
    
    /**
     * Process incoming serial data (call frequently from loop)
     * Non-blocking - processes all available bytes
     */
    void process();
    
    /**
     * Set callback for received frames
     * @param callback Function to call when frame is complete
     */
    void setFrameCallback(FrameCallback callback);
    
    /**
     * Check if serial data has been received recently
     * @return true if data received within TEST_MODE_TIMEOUT_MS
     */
    bool isReceiving() const;
    
    /**
     * Get time since last valid frame
     * @return Milliseconds since last frame, or UINT32_MAX if none
     */
    uint32_t timeSinceLastFrame() const;
    
    /**
     * Get statistics
     */
    const SerialRxStats& getStats() const { return stats_; }
    
    /**
     * Reset statistics
     */
    void resetStats() { stats_.reset(); }
    
    /**
     * Get current state (for debugging)
     */
    SerialRxState getState() const { return state_; }

private:
    HardwareSerial* serial_;
    SerialRxState state_;
    FrameCallback callback_;
    SerialRxStats stats_;
    
    // Receive buffer
    SerialDmxFrame frame_;
    uint16_t buffer_index_;
    
    // Timing
    unsigned long last_byte_time_;
    
    // State machine handlers
    void handleByte(uint8_t byte);
    void resync();
    void frameComplete();
    bool validateCrc();
};

#endif // AETHER_SERIAL_RX_H
