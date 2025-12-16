/**
 * AETHER ESP-NOW Node - ESP-NOW Receiver
 * 
 * Receives chunked DMX frames from Gateway over ESP-NOW.
 * Handles packet validation, reassembly, and sequence tracking.
 */

#ifndef AETHER_ESPNOW_RX_H
#define AETHER_ESPNOW_RX_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "../common/config.h"
#include "../common/packet.h"
#include "../common/crc16.h"

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

struct EspNowRxStats {
    uint32_t packets_received;      // Total ESP-NOW packets
    uint32_t packets_valid;         // Packets passing CRC/magic check
    uint32_t packets_invalid;       // Failed validation
    uint32_t frames_complete;       // Full frames reassembled
    uint32_t frames_dropped;        // Frames with missing chunks
    uint32_t sequence_resets;       // Sequence number jumps
    int8_t   last_rssi;             // Last packet RSSI
    unsigned long last_packet_time; // millis() of last valid packet
    
    void reset() {
        packets_received = 0;
        packets_valid = 0;
        packets_invalid = 0;
        frames_complete = 0;
        frames_dropped = 0;
        sequence_resets = 0;
        last_rssi = 0;
        last_packet_time = 0;
    }
};

// ═══════════════════════════════════════════════════════════════════
// FRAME REASSEMBLY BUFFER
// ═══════════════════════════════════════════════════════════════════

struct FrameReassembly {
    uint16_t universe;
    uint32_t seq;
    uint8_t chunks_received;            // Bitmask of received chunks
    uint8_t chunk_total;
    uint8_t data[DMX_UNIVERSE_SIZE];
    unsigned long start_time;
    bool active;
    
    void reset() {
        universe = 0;
        seq = 0;
        chunks_received = 0;
        chunk_total = CHUNKS_PER_FRAME;
        memset(data, 0, DMX_UNIVERSE_SIZE);
        start_time = 0;
        active = false;
    }
    
    bool isComplete() const {
        // Check if all chunks have been received
        uint8_t expected = (1 << chunk_total) - 1;  // e.g., 0b111 for 3 chunks
        return (chunks_received & expected) == expected;
    }
    
    bool hasTimedOut(unsigned long now) const {
        return active && (now - start_time) > CHUNK_TIMEOUT_MS;
    }
};

// ═══════════════════════════════════════════════════════════════════
// ESP-NOW RECEIVER CLASS
// ═══════════════════════════════════════════════════════════════════

class EspNowReceiver {
public:
    // Callback type for when a complete frame is reassembled
    using FrameCallback = void (*)(uint16_t universe, uint32_t seq, 
                                   const uint8_t* dmx_data);

    EspNowReceiver();
    
    /**
     * Initialize ESP-NOW in receive mode
     * @param universe Universe to listen for (0 = all)
     * @return true if successful
     */
    bool begin(uint16_t universe = 0);
    
    /**
     * Process timeouts and housekeeping (call from loop)
     */
    void process();
    
    /**
     * Set callback for completed frames
     * @param callback Function to call when frame is complete
     */
    void setFrameCallback(FrameCallback callback);
    
    /**
     * Set the universe filter
     * @param universe Universe to accept (0 = all)
     */
    void setUniverse(uint16_t universe);
    
    /**
     * Get the configured universe
     */
    uint16_t getUniverse() const { return universe_filter_; }
    
    /**
     * Check if data has been received recently
     * @return true if packet received within timeout
     */
    bool isReceiving() const;
    
    /**
     * Get time since last packet
     */
    uint32_t timeSinceLastPacket() const;
    
    /**
     * Get statistics
     */
    const EspNowRxStats& getStats() const { return stats_; }
    
    /**
     * Reset statistics
     */
    void resetStats() { stats_.reset(); }
    
    /**
     * Get last received RSSI
     */
    int8_t getLastRssi() const { return stats_.last_rssi; }

private:
    bool initialized_;
    uint16_t universe_filter_;
    uint8_t channel_;
    FrameCallback callback_;
    EspNowRxStats stats_;
    
    // Frame reassembly buffer
    FrameReassembly reassembly_;
    
    // Last known sequence number for duplicate detection
    uint32_t last_seq_;
    
    // Static callback for ESP-NOW API
    static void onReceiveCallback(const esp_now_recv_info_t* info,
                                   const uint8_t* data, int len);
    static EspNowReceiver* instance_;
    
    // Internal packet handler
    void handlePacket(const uint8_t* data, int len, int8_t rssi);
    void handleChunk(const DmxNowPacket& packet);
    void frameComplete();
    void checkTimeout();
};

// Global instance
extern EspNowReceiver espNowRx;

#endif // AETHER_ESPNOW_RX_H
