/**
 * AETHER Pulse - Hybrid Transport Implementation
 *
 * Transport: sACN/E1.31 multicast + UDP JSON commands
 * Fade model: Node-side (JSON commands include fade_ms)
 *
 * Only compiled when ENABLE_SACN is defined.
 */

#ifdef ENABLE_SACN

#include "transport.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncE131.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

#ifndef UDP_CONFIG_PORT
#define UDP_CONFIG_PORT 8888
#endif

#ifndef SACN_TIMEOUT_MS
#define SACN_TIMEOUT_MS 5000
#endif

// ═══════════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════════

static ESPAsyncE131* e131 = nullptr;
static WiFiUDP configUdp;
static uint16_t currentUniverse = 1;
static TransportFrameCallback frameCallback = nullptr;
static TransportCommandCallback commandCallback = nullptr;
static uint32_t lastSacnTime = 0;

TransportStats transportStats;

// ═══════════════════════════════════════════════════════════════════
// IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════

namespace Transport {

bool init(uint16_t universe) {
    currentUniverse = universe;
    transportStats.reset();

    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Transport: WiFi not connected");
        return false;
    }

    // Initialize sACN/E1.31
    if (e131 != nullptr) {
        delete e131;
    }
    e131 = new ESPAsyncE131(1);

    if (!e131->begin(E131_MULTICAST, currentUniverse, 1)) {
        Serial.println("Transport: sACN init failed");
        return false;
    }

    // Initialize UDP for JSON commands
    configUdp.begin(UDP_CONFIG_PORT);

    Serial.printf("Transport: Hybrid mode (sACN U%d + UDP:%d)\n",
                  currentUniverse, UDP_CONFIG_PORT);
    return true;
}

void setUniverse(uint16_t universe) {
    if (universe == currentUniverse) return;

    currentUniverse = universe;

    // Reinitialize sACN with new universe
    if (e131 != nullptr) {
        delete e131;
        e131 = new ESPAsyncE131(1);
        e131->begin(E131_MULTICAST, currentUniverse, 1);
    }

    Serial.printf("Transport: Changed to universe %d\n", currentUniverse);
}

void onFrame(TransportFrameCallback callback) {
    frameCallback = callback;
}

void onCommand(TransportCommandCallback callback) {
    commandCallback = callback;
}

void poll() {
    // Poll sACN packets
    if (e131 != nullptr && !e131->isEmpty()) {
        e131_packet_t packet;
        e131->pull(&packet);

        transportStats.packets_received++;
        transportStats.last_packet_time = millis();
        lastSacnTime = millis();

        // sACN frames don't include fade - snap immediately
        if (frameCallback) {
            // property_values[0] is start code, channels start at index 1
            frameCallback(currentUniverse, packet.property_values + 1, 0, packet.sequence_number);
        }
    }

    // Poll UDP JSON commands
    int packetSize = configUdp.parsePacket();
    if (packetSize > 0) {
        char buffer[2500];
        int len = configUdp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';
            transportStats.packets_received++;
            transportStats.bytes_received += len;

            if (commandCallback) {
                commandCallback(String(buffer));
            }
        }
    }
}

uint32_t getPacketsReceived() {
    return transportStats.packets_received;
}

uint32_t getPacketsDropped() {
    return transportStats.packets_dropped;
}

int8_t getLastRssi() {
    return WiFi.RSSI();
}

const char* getModeName() {
    return "sACN+UDP";
}

bool isConnected() {
    if (WiFi.status() != WL_CONNECTED) return false;
    // Consider connected if we've received a sACN packet recently
    return (millis() - lastSacnTime) < SACN_TIMEOUT_MS;
}

} // namespace Transport

#endif // ENABLE_SACN
