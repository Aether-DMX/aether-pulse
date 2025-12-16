/**
 * AETHER ESP-NOW Gateway - ESP-NOW Transmitter
 * 
 * Chunks DMX frames and broadcasts over ESP-NOW to all nodes.
 */

#ifndef AETHER_ESPNOW_TX_H
#define AETHER_ESPNOW_TX_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "../common/config.h"
#include "../common/packet.h"
#include "../common/crc16.h"

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

struct EspNowTxStats {
    uint32_t frames_sent;       // Complete DMX frames sent
    uint32_t packets_sent;      // Individual ESP-NOW packets
    uint32_t send_failures;     // esp_now_send() failures
    uint32_t callback_errors;   // Send callback reported errors
    unsigned long last_send_time;
    
    void reset() {
        frames_sent = 0;
        packets_sent = 0;
        send_failures = 0;
        callback_errors = 0;
        last_send_time = 0;
    }
};

// ═══════════════════════════════════════════════════════════════════
// ESP-NOW TRANSMITTER CLASS
// ═══════════════════════════════════════════════════════════════════

class EspNowTransmitter {
public:
    EspNowTransmitter();
    
    /**
     * Initialize ESP-NOW in transmit mode
     * @return true if successful
     */
    bool begin();
    
    /**
     * Send a complete DMX frame (will be chunked automatically)
     * @param universe DMX universe ID
     * @param seq Frame sequence number
     * @param dmx_data DMX channel data (512 bytes)
     * @return true if all chunks sent successfully
     */
    bool sendFrame(uint16_t universe, uint32_t seq, const uint8_t* dmx_data);
    
    /**
     * Check if ESP-NOW is initialized and ready
     */
    bool isReady() const { return initialized_; }
    
    /**
     * Get statistics
     */
    const EspNowTxStats& getStats() const { return stats_; }
    
    /**
     * Reset statistics
     */
    void resetStats() { stats_.reset(); }
    
    /**
     * Get current Wi-Fi channel
     */
    uint8_t getChannel() const { return channel_; }

private:
    bool initialized_;
    uint8_t channel_;
    EspNowTxStats stats_;
    
    // Packet buffer for sending
    DmxNowPacket packet_;
    
    // Send callback (static for ESP-NOW API)
    static void onSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status);
    static EspNowTransmitter* instance_;
    
    void handleSendResult(esp_now_send_status_t status);
    
    // Build and send a single chunk
    bool sendChunk(uint16_t universe, uint32_t seq, 
                   uint8_t chunk_index, const uint8_t* data, uint16_t data_len);
};

// Global instance
extern EspNowTransmitter espNowTx;

#endif // AETHER_ESPNOW_TX_H
