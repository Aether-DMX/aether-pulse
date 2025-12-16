/**
 * AETHER ESP-NOW DMX - Shared Configuration
 * 
 * Common settings for both Gateway and Node firmware.
 * Edit these values to customize your deployment.
 */

#ifndef AETHER_ESPNOW_CONFIG_H
#define AETHER_ESPNOW_CONFIG_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
// VERSION
// ═══════════════════════════════════════════════════════════════════
#define AETHER_ESPNOW_VERSION_MAJOR 1
#define AETHER_ESPNOW_VERSION_MINOR 0
#define AETHER_ESPNOW_VERSION_PATCH 0
#define AETHER_ESPNOW_VERSION_STRING "1.0.0"

// ═══════════════════════════════════════════════════════════════════
// ESP-NOW CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

// Wi-Fi channel for ESP-NOW (1-13, must match on all devices)
// Channel 1 or 6 recommended for best compatibility
#ifndef ESPNOW_CHANNEL
#define ESPNOW_CHANNEL 6
#endif

// Broadcast address for ESP-NOW
static const uint8_t ESPNOW_BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Maximum ESP-NOW payload size (250 bytes is safe limit)
#define ESPNOW_MAX_PAYLOAD 250

// ═══════════════════════════════════════════════════════════════════
// DMX CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

// DMX universe size (always 512 for DMX512)
#define DMX_UNIVERSE_SIZE 512

// DMX packet size including start code
#define DMX_PACKET_SIZE 513

// Default DMX output rate in Hz
#define DMX_REFRESH_HZ 40

// DMX timing (microseconds)
#define DMX_BREAK_US 176
#define DMX_MAB_US 12

// Default universe assignment
#ifndef DEFAULT_UNIVERSE
#define DEFAULT_UNIVERSE 1
#endif

// ═══════════════════════════════════════════════════════════════════
// PACKET CHUNKING
// ═══════════════════════════════════════════════════════════════════

// Data bytes per chunk (must fit in ESP-NOW payload with headers)
// Header is ~16 bytes, CRC is 2 bytes, so ~200 bytes for data is safe
#define CHUNK_DATA_SIZE 200

// Number of chunks needed for full DMX frame
// ceil(512 / 200) = 3
#define CHUNKS_PER_FRAME 3

// Chunk reassembly timeout (ms) - discard incomplete frames after this
#define CHUNK_TIMEOUT_MS 50

// ═══════════════════════════════════════════════════════════════════
// PACKET MAGIC & ROLES
// ═══════════════════════════════════════════════════════════════════

// Magic bytes: 'A', 'X' for "Aether Xmit"
#define PACKET_MAGIC_0 0x41  // 'A'
#define PACKET_MAGIC_1 0x58  // 'X'

// Protocol version
#define PACKET_VERSION 1

// Device roles
#define ROLE_GATEWAY 1
#define ROLE_NODE    2

// ═══════════════════════════════════════════════════════════════════
// SERIAL PROTOCOL (Pi → Gateway)
// ═══════════════════════════════════════════════════════════════════

// Serial baud rate
#define SERIAL_BAUD_RATE 921600

// Preamble bytes
#define SERIAL_PREAMBLE_0 0xAA
#define SERIAL_PREAMBLE_1 0x55

// Maximum time between bytes before resync (ms)
#define SERIAL_TIMEOUT_MS 100

// ═══════════════════════════════════════════════════════════════════
// HARDWARE PINS - NODE
// ═══════════════════════════════════════════════════════════════════

// DMX output pins (RS485)
#define DMX_TX_PIN 17
#define DMX_RX_PIN 16
#define DMX_ENABLE_PIN 4
#define DMX_PORT 1

// Status LED
#define LED_PIN 2

// OLED display (I2C)
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// ═══════════════════════════════════════════════════════════════════
// TEST PATTERN (Gateway)
// ═══════════════════════════════════════════════════════════════════

#ifndef TEST_PATTERN_ENABLED
#define TEST_PATTERN_ENABLED 1
#endif

// Test pattern types
#define TEST_PATTERN_SINE    0
#define TEST_PATTERN_CHASE   1
#define TEST_PATTERN_FULL    2
#define TEST_PATTERN_RAMP    3

// Default test pattern
#ifndef TEST_PATTERN_MODE
#define TEST_PATTERN_MODE TEST_PATTERN_SINE
#endif

// Test pattern rate (ms between updates)
#define TEST_PATTERN_INTERVAL_MS 25

// Time without serial data before entering test mode (ms)
#define TEST_MODE_TIMEOUT_MS 5000

// ═══════════════════════════════════════════════════════════════════
// RELIABILITY & TIMING
// ═══════════════════════════════════════════════════════════════════

// Maximum sequence number before wrap
#define SEQ_MAX 0xFFFFFFFF

// How many old sequence numbers to accept (for out-of-order packets)
#define SEQ_WINDOW 10

// Heartbeat interval for node stats (ms)
#define HEARTBEAT_INTERVAL_MS 10000

// Hold-last-look timeout (ms) - how long to hold output after signal loss
// Set to 0 for infinite hold
#define HOLD_LAST_LOOK_TIMEOUT_MS 0

// Blackout fade time when signal lost (ms) - only if HOLD_LAST_LOOK_TIMEOUT > 0
#define SIGNAL_LOSS_FADE_MS 2000

// ═══════════════════════════════════════════════════════════════════
// DEBUG FLAGS
// ═══════════════════════════════════════════════════════════════════

#ifndef DEBUG_VERBOSE
#define DEBUG_VERBOSE 0
#endif

#ifndef DEBUG_PACKET_HEX
#define DEBUG_PACKET_HEX 0
#endif

// Debug print interval (ms)
#define DEBUG_PRINT_INTERVAL_MS 5000

// ═══════════════════════════════════════════════════════════════════
// STATISTICS
// ═══════════════════════════════════════════════════════════════════

// Enable statistics tracking
#define STATS_ENABLED 1

// Stats calculation window (ms)
#define STATS_WINDOW_MS 1000

#endif // AETHER_ESPNOW_CONFIG_H
