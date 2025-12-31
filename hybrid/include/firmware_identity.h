/**
 * AETHER Pulse Firmware - Identity & Version Info
 *
 * Single transport: sACN/E1.31 over WiFi
 */

#ifndef AETHER_FIRMWARE_IDENTITY_H
#define AETHER_FIRMWARE_IDENTITY_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
// VERSION
// ═══════════════════════════════════════════════════════════════════

#ifndef AETHER_FIRMWARE_VERSION_MAJOR
#define AETHER_FIRMWARE_VERSION_MAJOR 2
#endif

#ifndef AETHER_FIRMWARE_VERSION_MINOR
#define AETHER_FIRMWARE_VERSION_MINOR 2
#endif

#ifndef AETHER_FIRMWARE_VERSION_PATCH
#define AETHER_FIRMWARE_VERSION_PATCH 0
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define AETHER_VERSION_STRING \
    TOSTRING(AETHER_FIRMWARE_VERSION_MAJOR) "." \
    TOSTRING(AETHER_FIRMWARE_VERSION_MINOR) "." \
    TOSTRING(AETHER_FIRMWARE_VERSION_PATCH)

// ═══════════════════════════════════════════════════════════════════
// FIRMWARE IDENTITY (sACN only - no legacy transports)
// ═══════════════════════════════════════════════════════════════════

#define AETHER_FAMILY_NAME "pulse-sacn"
#define AETHER_FAMILY_ID 1
#define AETHER_TRANSPORT "sACN"

#ifndef AETHER_OTA_CHANNEL
#define AETHER_OTA_CHANNEL "stable"
#endif

// ═══════════════════════════════════════════════════════════════════
// BOOT BANNER
// ═══════════════════════════════════════════════════════════════════

inline void printFirmwareIdentity(const char* node_id, int universe) {
    Serial.println();
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println("  AETHER Pulse - sACN/E1.31 DMX Node");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("  Firmware:  %s v%s\n", AETHER_FAMILY_NAME, AETHER_VERSION_STRING);
    Serial.printf("  Transport: %s (WiFi multicast)\n", AETHER_TRANSPORT);
    Serial.printf("  Node ID:   %s\n", node_id);
    Serial.printf("  Universe:  %d\n", universe);
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// STATUS JSON
// ═══════════════════════════════════════════════════════════════════

inline String buildStatusJson(const char* node_id, int universe,
                               uint32_t rx_packets, uint32_t tx_frames,
                               int rssi, bool receiving, uint32_t last_sacn_ms) {
    char json[512];
    snprintf(json, sizeof(json),
        "{"
        "\"firmware\":\"%s\","
        "\"version\":\"%s\","
        "\"transport\":\"%s\","
        "\"node_id\":\"%s\","
        "\"universe\":%d,"
        "\"sacn_packets\":%lu,"
        "\"dmx_frames\":%lu,"
        "\"rssi\":%d,"
        "\"receiving\":%s,"
        "\"last_sacn_ms\":%lu"
        "}",
        AETHER_FAMILY_NAME,
        AETHER_VERSION_STRING,
        AETHER_TRANSPORT,
        node_id,
        universe,
        rx_packets,
        tx_frames,
        rssi,
        receiving ? "true" : "false",
        last_sacn_ms
    );
    return String(json);
}

#endif // AETHER_FIRMWARE_IDENTITY_H
