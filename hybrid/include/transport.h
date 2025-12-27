/**
 * AETHER Pulse - Transport Layer Interface
 *
 * Abstraction layer for DMX frame input transport.
 * Implementations:
 *   - transport_hybrid.cpp: sACN/E1.31 + UDP JSON
 *   - transport_espnow.cpp: ESP-NOW broadcast
 */

#ifndef AETHER_TRANSPORT_H
#define AETHER_TRANSPORT_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
// TRANSPORT CALLBACK TYPES
// ═══════════════════════════════════════════════════════════════════

/**
 * Callback for received DMX frame
 * @param universe DMX universe ID
 * @param channels DMX channel data (512 bytes, index 0 = channel 1)
 * @param fade_ms Fade time in milliseconds (0 = instant)
 * @param seq Frame sequence number (for ordering/dedup)
 */
typedef void (*TransportFrameCallback)(uint16_t universe, const uint8_t* channels,
                                        uint32_t fade_ms, uint32_t seq);

/**
 * Callback for received JSON command
 * @param json JSON command string
 */
typedef void (*TransportCommandCallback)(const String& json);

// ═══════════════════════════════════════════════════════════════════
// TRANSPORT INTERFACE
// ═══════════════════════════════════════════════════════════════════

namespace Transport {

/**
 * Initialize the transport layer
 * @param universe Initial universe to subscribe to
 * @return true on success
 */
bool init(uint16_t universe);

/**
 * Change universe subscription
 * @param universe New universe ID
 */
void setUniverse(uint16_t universe);

/**
 * Register callback for DMX frames
 */
void onFrame(TransportFrameCallback callback);

/**
 * Register callback for JSON commands
 */
void onCommand(TransportCommandCallback callback);

/**
 * Process incoming packets (call from loop())
 */
void poll();

/**
 * Get transport statistics
 */
uint32_t getPacketsReceived();
uint32_t getPacketsDropped();
int8_t getLastRssi();

/**
 * Get transport mode name
 */
const char* getModeName();

/**
 * Check if transport is connected/healthy
 */
bool isConnected();

} // namespace Transport

// ═══════════════════════════════════════════════════════════════════
// ESP-NOW GATEWAY / REBROADCAST INTERFACE
// ═══════════════════════════════════════════════════════════════════
// When ENABLE_ESPNOW_GATEWAY is defined, these functions allow
// rebroadcasting received DMX frames over ESP-NOW to other nodes.
// This enables a hybrid architecture where:
//   - Gateway receives via sACN/WiFi from Pi
//   - Gateway rebroadcasts via ESP-NOW to ESP-NOW nodes
// ═══════════════════════════════════════════════════════════════════

#ifdef ENABLE_ESPNOW_GATEWAY

namespace EspNowGateway {

/**
 * Initialize ESP-NOW in gateway/transmit mode
 * Can be called alongside WiFi (uses same radio, different protocol)
 * @return true on success
 */
bool init();

/**
 * Broadcast a DMX frame to all ESP-NOW nodes
 * @param universe DMX universe ID
 * @param channels DMX channel data (512 bytes)
 * @param fade_ms Fade time in milliseconds
 * @param seq Frame sequence number
 * @return true if broadcast was sent
 */
bool broadcastFrame(uint16_t universe, const uint8_t* channels,
                    uint32_t fade_ms, uint32_t seq);

/**
 * Get gateway statistics
 */
uint32_t getFramesSent();
uint32_t getSendErrors();

} // namespace EspNowGateway

#endif // ENABLE_ESPNOW_GATEWAY

// ═══════════════════════════════════════════════════════════════════
// TRANSPORT STATISTICS
// ═══════════════════════════════════════════════════════════════════

struct TransportStats {
    uint32_t packets_received;
    uint32_t packets_dropped;
    uint32_t bytes_received;
    uint32_t last_seq;
    int8_t last_rssi;
    uint32_t last_packet_time;

    void reset() {
        packets_received = 0;
        packets_dropped = 0;
        bytes_received = 0;
        last_seq = 0;
        last_rssi = 0;
        last_packet_time = 0;
    }
};

// Global stats instance (defined in transport implementation)
extern TransportStats transportStats;

#endif // AETHER_TRANSPORT_H
