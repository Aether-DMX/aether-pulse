/**
 * AETHER Pulse - ESP-NOW Gateway/Rebroadcast Implementation
 *
 * Enables a node to rebroadcast received DMX frames over ESP-NOW.
 * Used when running alongside WiFi/sACN to bridge to ESP-NOW nodes.
 *
 * Only compiled when ENABLE_ESPNOW_GATEWAY is defined.
 *
 * Architecture:
 *   Pi → (sACN/WiFi) → Gateway ESP → (ESP-NOW) → Node ESPs
 *
 * Packet format matches transport_espnow.cpp receiver expectations.
 */

#ifdef ENABLE_ESPNOW_GATEWAY

#include "transport.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 6
#endif

#define ESPNOW_MAGIC_0 'A'
#define ESPNOW_MAGIC_1 'D'
#define ESPNOW_VERSION 1

// Chunk size for splitting 512-byte DMX frame
// ESP-NOW max payload is ~250 bytes, we use 200 for safety
#define CHUNK_SIZE 200
#define NUM_CHUNKS 3  // ceil(512/200) = 3

// Broadcast address
static const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ═══════════════════════════════════════════════════════════════════
// PACKET STRUCTURE (must match transport_espnow.cpp)
// ═══════════════════════════════════════════════════════════════════

#pragma pack(push, 1)
struct EspNowDmxPacket {
    uint8_t magic[2];       // 'A', 'D'
    uint8_t version;        // Protocol version
    uint16_t universe;      // DMX universe
    uint32_t seq;           // Sequence number
    uint16_t fade_ms;       // Fade time
    uint8_t chunk_idx;      // Chunk index (0, 1, 2)
    uint8_t chunk_tot;      // Total chunks (3)
    uint16_t offset;        // Start channel in this chunk
    uint8_t length;         // Channels in this chunk
    uint8_t data[CHUNK_SIZE]; // Channel values
};
#pragma pack(pop)

// ═══════════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════════

static bool gatewayInitialized = false;
static uint32_t framesSent = 0;
static uint32_t sendErrors = 0;
static volatile bool lastSendOk = true;

// Packet buffer
static EspNowDmxPacket txPacket;

// ═══════════════════════════════════════════════════════════════════
// SEND CALLBACK
// ═══════════════════════════════════════════════════════════════════

static void onSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        lastSendOk = false;
    }
}

// ═══════════════════════════════════════════════════════════════════
// IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════

namespace EspNowGateway {

bool init() {
    if (gatewayInitialized) {
        return true;
    }

    // ESP-NOW can run alongside WiFi STA mode
    // WiFi should already be initialized by main code

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("EspNowGateway: ESP-NOW init failed");
        return false;
    }

    // Register send callback
    esp_now_register_send_cb(onSendCallback);

    // Add broadcast peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADDR, 6);
    peerInfo.channel = 0;  // Use current WiFi channel
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("EspNowGateway: Failed to add broadcast peer");
        // Continue anyway - might already exist
    }

    // Initialize packet template
    txPacket.magic[0] = ESPNOW_MAGIC_0;
    txPacket.magic[1] = ESPNOW_MAGIC_1;
    txPacket.version = ESPNOW_VERSION;
    txPacket.chunk_tot = NUM_CHUNKS;

    gatewayInitialized = true;

    // Get current WiFi channel
    uint8_t primaryChan;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);

    Serial.printf("EspNowGateway: Initialized (WiFi ch %d)\n", primaryChan);
    return true;
}

bool broadcastFrame(uint16_t universe, const uint8_t* channels,
                    uint32_t fade_ms, uint32_t seq) {
    if (!gatewayInitialized) {
        return false;
    }

    txPacket.universe = universe;
    txPacket.seq = seq;
    txPacket.fade_ms = fade_ms;

    bool allSent = true;

    // Send in chunks
    for (uint8_t chunk = 0; chunk < NUM_CHUNKS; chunk++) {
        uint16_t offset = chunk * CHUNK_SIZE;
        uint8_t length = CHUNK_SIZE;

        // Last chunk may be shorter
        if (offset + length > 512) {
            length = 512 - offset;
        }

        txPacket.chunk_idx = chunk;
        txPacket.offset = offset;
        txPacket.length = length;

        // Copy channel data
        memcpy(txPacket.data, channels + offset, length);

        // Calculate actual packet size (header + data, no padding)
        size_t packetSize = 16 + length;  // Header is 16 bytes

        lastSendOk = true;
        esp_err_t result = esp_now_send(BROADCAST_ADDR, (uint8_t*)&txPacket, packetSize);

        if (result != ESP_OK) {
            sendErrors++;
            allSent = false;
        }

        // Small delay between chunks to avoid overwhelming receivers
        delayMicroseconds(200);
    }

    if (allSent) {
        framesSent++;
    }

    return allSent;
}

uint32_t getFramesSent() {
    return framesSent;
}

uint32_t getSendErrors() {
    return sendErrors;
}

} // namespace EspNowGateway

#endif // ENABLE_ESPNOW_GATEWAY
