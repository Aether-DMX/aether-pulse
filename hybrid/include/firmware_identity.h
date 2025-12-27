/**
 * AETHER Pulse Firmware - Identity & OTA Safety
 *
 * This header provides firmware identification and OTA channel gating.
 * CRITICAL: This prevents hybrid nodes from accepting espnow firmware and vice versa.
 */

#ifndef AETHER_FIRMWARE_IDENTITY_H
#define AETHER_FIRMWARE_IDENTITY_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
// VERSION STRING HELPERS
// ═══════════════════════════════════════════════════════════════════

#ifndef AETHER_FIRMWARE_VERSION_MAJOR
#define AETHER_FIRMWARE_VERSION_MAJOR 1
#endif

#ifndef AETHER_FIRMWARE_VERSION_MINOR
#define AETHER_FIRMWARE_VERSION_MINOR 3
#endif

#ifndef AETHER_FIRMWARE_VERSION_PATCH
#define AETHER_FIRMWARE_VERSION_PATCH 0
#endif

// Stringify helpers
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Version string: "1.3.0"
#define AETHER_VERSION_STRING \
    TOSTRING(AETHER_FIRMWARE_VERSION_MAJOR) "." \
    TOSTRING(AETHER_FIRMWARE_VERSION_MINOR) "." \
    TOSTRING(AETHER_FIRMWARE_VERSION_PATCH)

// ═══════════════════════════════════════════════════════════════════
// FIRMWARE FAMILY DETECTION
// ═══════════════════════════════════════════════════════════════════

#if defined(PULSE_FAMILY_HYBRID)
    #define AETHER_FAMILY_NAME "pulse-hybrid"
    #define AETHER_FAMILY_ID 1
#elif defined(PULSE_FAMILY_ESPNOW)
    #define AETHER_FAMILY_NAME "pulse-espnow"
    #define AETHER_FAMILY_ID 2
#else
    #error "Must define PULSE_FAMILY_HYBRID or PULSE_FAMILY_ESPNOW"
#endif

// ═══════════════════════════════════════════════════════════════════
// OTA CHANNEL SAFETY
// ═══════════════════════════════════════════════════════════════════
// Firmware will REFUSE updates from a different family unless explicitly
// opted-in via a special flag in the OTA manifest.

#ifndef AETHER_OTA_CHANNEL
    #if defined(PULSE_FAMILY_HYBRID)
        #define AETHER_OTA_CHANNEL "stable-hybrid"
    #elif defined(PULSE_FAMILY_ESPNOW)
        #define AETHER_OTA_CHANNEL "beta-espnow"
    #endif
#endif

/**
 * Check if an OTA update should be accepted based on channel matching.
 *
 * @param manifest_family The firmware_family from OTA manifest
 * @param manifest_channel The ota_channel from OTA manifest
 * @param force_crossover If true, allow cross-family update (dangerous)
 * @return true if update should proceed, false to reject
 */
inline bool shouldAcceptOtaUpdate(const char* manifest_family,
                                    const char* manifest_channel,
                                    bool force_crossover = false) {
    // If force_crossover is set, allow any update (for recovery/migration)
    if (force_crossover) {
        Serial.println("⚠️ OTA: Force crossover enabled - accepting update");
        return true;
    }

    // Check family match
    if (strcmp(manifest_family, AETHER_FAMILY_NAME) != 0) {
        Serial.printf("❌ OTA REJECTED: Family mismatch (running: %s, update: %s)\n",
                        AETHER_FAMILY_NAME, manifest_family);
        Serial.println("   To force crossover, include 'force_crossover: true' in manifest");
        return false;
    }

    // Family matches - allow update
    Serial.printf("✅ OTA: Family match (%s), accepting update\n", AETHER_FAMILY_NAME);
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// BOOT BANNER
// ═══════════════════════════════════════════════════════════════════

/**
 * Print firmware identity on boot (call from setup())
 */
inline void printFirmwareIdentity(const char* node_id, int universe) {
    Serial.println();
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println("  AETHER Pulse DMX Node");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("  Firmware:  %s v%s\n", AETHER_FAMILY_NAME, AETHER_VERSION_STRING);
    Serial.printf("  OTA Chan:  %s\n", AETHER_OTA_CHANNEL);
    Serial.printf("  Node ID:   %s\n", node_id);
    Serial.printf("  Universe:  %d\n", universe);
#if defined(ENABLE_ESPNOW_GATEWAY) && defined(ENABLE_SACN)
    Serial.println("  Transport: sACN/E1.31 + ESP-NOW Gateway");
    Serial.printf("  ESP-NOW:   Channel %d (rebroadcast mode)\n", ESPNOW_CHANNEL);
#elif defined(ENABLE_ESPNOW_GATEWAY)
    Serial.println("  Transport: UART + ESP-NOW Gateway");
    Serial.printf("  ESP-NOW:   Channel %d (rebroadcast mode)\n", ESPNOW_CHANNEL);
#elif defined(ENABLE_SACN)
    Serial.println("  Transport: sACN/E1.31 + UDP JSON");
#elif defined(ENABLE_ESPNOW)
    Serial.printf("  Transport: ESP-NOW (channel %d)\n", ESPNOW_CHANNEL);
#endif
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// STATUS RESPONSE
// ═══════════════════════════════════════════════════════════════════

/**
 * Build JSON status response for status endpoint/UDP reply
 */
inline String buildStatusJson(const char* node_id, int universe,
                               uint32_t rx_packets, uint32_t tx_frames,
                               int rssi, bool fade_active, uint32_t fade_ms,
                               uint32_t last_seq) {
    char json[512];
    snprintf(json, sizeof(json),
        "{"
        "\"firmware_family\":\"%s\","
        "\"version\":\"%s\","
        "\"ota_channel\":\"%s\","
        "\"node_id\":\"%s\","
        "\"universe\":%d,"
        "\"rx_packets_sec\":%lu,"
        "\"tx_frames_sec\":%lu,"
        "\"rssi\":%d,"
        "\"fade_active\":%s,"
        "\"fade_ms\":%lu,"
        "\"last_seq\":%lu"
        "}",
        AETHER_FAMILY_NAME,
        AETHER_VERSION_STRING,
        AETHER_OTA_CHANNEL,
        node_id,
        universe,
        rx_packets,
        tx_frames,
        rssi,
        fade_active ? "true" : "false",
        fade_ms,
        last_seq
    );
    return String(json);
}

#endif // AETHER_FIRMWARE_IDENTITY_H
