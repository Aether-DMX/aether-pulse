/**
 * AETHER Pulse - ESP-NOW Transport Implementation
 *
 * Transport: ESP-NOW broadcast from Gateway ESP32
 * Fade model: Node-side (packets include fade_ms)
 *
 * Only compiled when ENABLE_ESPNOW is defined.
 *
 * Packet format (from Gateway):
 *   [0-1]   magic      'A','D' (Aether DMX)
 *   [2]     version    Protocol version
 *   [3-4]   universe   DMX universe (little-endian)
 *   [5-8]   seq        Sequence number (little-endian)
 *   [9-10]  fade_ms    Fade time in ms (little-endian)
 *   [11]    chunk_idx  Chunk index (0,1,2 for 512 channels)
 *   [12]    chunk_tot  Total chunks
 *   [13-14] offset     Start channel in this chunk (little-endian)
 *   [15]    length     Channels in this chunk
 *   [16-N]  data       Channel values
 */

#ifdef ENABLE_ESPNOW

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

#define CHUNK_TIMEOUT_MS 50
#define MAX_CHUNKS 4

// ═══════════════════════════════════════════════════════════════════
// PACKET STRUCTURE
// ═══════════════════════════════════════════════════════════════════

#pragma pack(push, 1)
struct EspNowDmxPacket {
    uint8_t magic[2];       // 'A', 'D'
    uint8_t version;        // Protocol version
    uint16_t universe;      // DMX universe
    uint32_t seq;           // Sequence number
    uint16_t fade_ms;       // Fade time
    uint8_t chunk_idx;      // Chunk index
    uint8_t chunk_tot;      // Total chunks
    uint16_t offset;        // Start channel
    uint8_t length;         // Channels in this chunk
    uint8_t data[200];      // Channel values
};
#pragma pack(pop)

// ═══════════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════════

static uint16_t currentUniverse = 1;
static TransportFrameCallback frameCallback = nullptr;
static TransportCommandCallback commandCallback = nullptr;

// Frame reassembly buffer
static uint8_t frameBuffer[512];
static uint8_t chunksReceived[MAX_CHUNKS];
static uint32_t currentFrameSeq = 0;
static uint32_t frameStartTime = 0;
static uint8_t expectedChunks = 0;
static uint16_t pendingFadeMs = 0;
static volatile bool frameReady = false;
static volatile int8_t lastRssi = 0;

TransportStats transportStats;

// ═══════════════════════════════════════════════════════════════════
// ESP-NOW RECEIVE CALLBACK
// ═══════════════════════════════════════════════════════════════════

static void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len < 16) return; // Minimum packet size

    const EspNowDmxPacket* pkt = (const EspNowDmxPacket*)data;

    // Validate magic and version
    if (pkt->magic[0] != ESPNOW_MAGIC_0 || pkt->magic[1] != ESPNOW_MAGIC_1) {
        return;
    }
    if (pkt->version != ESPNOW_VERSION) {
        transportStats.packets_dropped++;
        return;
    }

    // Filter by universe
    if (pkt->universe != currentUniverse) {
        return;
    }

    transportStats.packets_received++;
    transportStats.last_packet_time = millis();

    // New frame sequence?
    if (pkt->seq != currentFrameSeq) {
        // If we had an incomplete frame, count it as dropped
        if (currentFrameSeq > 0 && expectedChunks > 0) {
            uint8_t received = 0;
            for (int i = 0; i < expectedChunks; i++) {
                if (chunksReceived[i]) received++;
            }
            if (received < expectedChunks) {
                transportStats.packets_dropped++;
            }
        }

        // Start new frame
        currentFrameSeq = pkt->seq;
        expectedChunks = pkt->chunk_tot;
        pendingFadeMs = pkt->fade_ms;
        frameStartTime = millis();
        memset(chunksReceived, 0, sizeof(chunksReceived));
        memset(frameBuffer, 0, sizeof(frameBuffer));
    }

    // Check chunk timeout
    if (millis() - frameStartTime > CHUNK_TIMEOUT_MS) {
        // Timed out, start fresh
        currentFrameSeq = pkt->seq;
        expectedChunks = pkt->chunk_tot;
        pendingFadeMs = pkt->fade_ms;
        frameStartTime = millis();
        memset(chunksReceived, 0, sizeof(chunksReceived));
        memset(frameBuffer, 0, sizeof(frameBuffer));
    }

    // Copy chunk data
    uint8_t idx = pkt->chunk_idx;
    if (idx < MAX_CHUNKS && !chunksReceived[idx]) {
        uint16_t offset = pkt->offset;
        uint8_t length = pkt->length;

        if (offset + length <= 512) {
            memcpy(frameBuffer + offset, pkt->data, length);
            chunksReceived[idx] = 1;
        }
    }

    // Check if frame complete
    uint8_t complete = 1;
    for (int i = 0; i < expectedChunks && i < MAX_CHUNKS; i++) {
        if (!chunksReceived[i]) {
            complete = 0;
            break;
        }
    }

    if (complete) {
        transportStats.last_seq = currentFrameSeq;
        frameReady = true;
    }
}

// ═══════════════════════════════════════════════════════════════════
// IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════

namespace Transport {

bool init(uint16_t universe) {
    currentUniverse = universe;
    transportStats.reset();

    // Initialize WiFi in station mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Set WiFi channel
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Transport: ESP-NOW init failed");
        return false;
    }

    // Register receive callback
    esp_now_register_recv_cb(onEspNowReceive);

    Serial.printf("Transport: ESP-NOW mode (channel %d, U%d)\n",
                  ESPNOW_CHANNEL, currentUniverse);
    return true;
}

void setUniverse(uint16_t universe) {
    currentUniverse = universe;
    Serial.printf("Transport: Changed to universe %d\n", currentUniverse);
}

void onFrame(TransportFrameCallback callback) {
    frameCallback = callback;
}

void onCommand(TransportCommandCallback callback) {
    commandCallback = callback;
}

void poll() {
    // Check for complete frame
    if (frameReady && frameCallback) {
        frameReady = false;
        frameCallback(currentUniverse, frameBuffer, pendingFadeMs, currentFrameSeq);

        // Reset for next frame
        currentFrameSeq = 0;
        expectedChunks = 0;
    }
}

uint32_t getPacketsReceived() {
    return transportStats.packets_received;
}

uint32_t getPacketsDropped() {
    return transportStats.packets_dropped;
}

int8_t getLastRssi() {
    return lastRssi;
}

const char* getModeName() {
    return "ESP-NOW";
}

bool isConnected() {
    // Consider connected if we've received a packet recently
    return (millis() - transportStats.last_packet_time) < 5000;
}

} // namespace Transport

#endif // ENABLE_ESPNOW
