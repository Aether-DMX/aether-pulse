/**
 * AETHER ESP-NOW DMX - Packet Structures
 * 
 * Defines the binary packet format for ESP-NOW transmission
 * and the serial protocol from Pi to Gateway.
 */

#ifndef AETHER_ESPNOW_PACKET_H
#define AETHER_ESPNOW_PACKET_H

#include <Arduino.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════════
// ESP-NOW DMX PACKET
// ═══════════════════════════════════════════════════════════════════
// 
// This packet is broadcast over ESP-NOW. DMX512 (512 bytes) is split
// into multiple chunks since ESP-NOW max payload is ~250 bytes.
//
// Layout:
//   [0-1]   magic      'A','X'
//   [2]     version    Protocol version (1)
//   [3]     role       1=gateway, 2=node
//   [4-5]   universe   DMX universe ID (little-endian)
//   [6-9]   seq        Frame sequence number (little-endian)
//   [10]    chunk_idx  Chunk index (0, 1, 2, ...)
//   [11]    chunk_tot  Total chunks in frame
//   [12-13] data_len   Bytes in this chunk (little-endian)
//   [14-N]  data       DMX channel data
//   [N+1-2] crc16      CRC16-CCITT of bytes [0-N] (little-endian)
//
// ═══════════════════════════════════════════════════════════════════

#pragma pack(push, 1)

struct DmxNowPacket {
    uint8_t  magic[2];              // 'A', 'X'
    uint8_t  version;               // Protocol version
    uint8_t  role;                  // Device role (gateway/node)
    uint16_t universe;              // DMX universe ID
    uint32_t seq;                   // Frame sequence number
    uint8_t  chunk_index;           // Chunk index (0-based)
    uint8_t  chunk_total;           // Total chunks in frame
    uint16_t data_len;              // Bytes of DMX data in this chunk
    uint8_t  data[CHUNK_DATA_SIZE]; // DMX channel data
    uint16_t crc16;                 // CRC16-CCITT
    
    // Initialize with defaults
    void init() {
        magic[0] = PACKET_MAGIC_0;
        magic[1] = PACKET_MAGIC_1;
        version = PACKET_VERSION;
        role = 0;
        universe = 0;
        seq = 0;
        chunk_index = 0;
        chunk_total = CHUNKS_PER_FRAME;
        data_len = 0;
        memset(data, 0, CHUNK_DATA_SIZE);
        crc16 = 0;
    }
    
    // Validate magic and version
    bool isValid() const {
        return magic[0] == PACKET_MAGIC_0 &&
               magic[1] == PACKET_MAGIC_1 &&
               version == PACKET_VERSION;
    }
    
    // Get total packet size (header + data + crc)
    size_t totalSize() const {
        // Header is 14 bytes (magic to data_len), data is variable, crc is 2 bytes
        return 14 + data_len + 2;
    }
    
    // Get header size (everything before data)
    static constexpr size_t headerSize() {
        return 14;
    }
};

#pragma pack(pop)

// Verify structure packing
static_assert(offsetof(DmxNowPacket, magic) == 0, "magic offset");
static_assert(offsetof(DmxNowPacket, version) == 2, "version offset");
static_assert(offsetof(DmxNowPacket, role) == 3, "role offset");
static_assert(offsetof(DmxNowPacket, universe) == 4, "universe offset");
static_assert(offsetof(DmxNowPacket, seq) == 6, "seq offset");
static_assert(offsetof(DmxNowPacket, chunk_index) == 10, "chunk_index offset");
static_assert(offsetof(DmxNowPacket, chunk_total) == 11, "chunk_total offset");
static_assert(offsetof(DmxNowPacket, data_len) == 12, "data_len offset");
static_assert(offsetof(DmxNowPacket, data) == 14, "data offset");

// ═══════════════════════════════════════════════════════════════════
// SERIAL FRAME (Pi → Gateway)
// ═══════════════════════════════════════════════════════════════════
//
// Binary frame format from Raspberry Pi to Gateway ESP32:
//
//   [0-1]   preamble   0xAA, 0x55
//   [2-3]   universe   DMX universe ID (little-endian)
//   [4-7]   seq        Sequence number (little-endian)
//   [8-9]   length     Payload length (little-endian, should be 512)
//   [10-N]  payload    DMX channel data (512 bytes)
//   [N+1-2] crc16      CRC16-CCITT of bytes [2-N] (little-endian)
//
// Total frame size: 2 + 2 + 4 + 2 + 512 + 2 = 524 bytes
//
// ═══════════════════════════════════════════════════════════════════

#pragma pack(push, 1)

struct SerialDmxFrame {
    uint8_t  preamble[2];           // 0xAA, 0x55
    uint16_t universe;              // DMX universe ID
    uint32_t seq;                   // Sequence number
    uint16_t length;                // Payload length
    uint8_t  payload[DMX_UNIVERSE_SIZE]; // DMX data
    uint16_t crc16;                 // CRC16-CCITT
    
    // Initialize with defaults
    void init() {
        preamble[0] = SERIAL_PREAMBLE_0;
        preamble[1] = SERIAL_PREAMBLE_1;
        universe = DEFAULT_UNIVERSE;
        seq = 0;
        length = DMX_UNIVERSE_SIZE;
        memset(payload, 0, DMX_UNIVERSE_SIZE);
        crc16 = 0;
    }
    
    // Validate preamble
    bool hasPreamble() const {
        return preamble[0] == SERIAL_PREAMBLE_0 &&
               preamble[1] == SERIAL_PREAMBLE_1;
    }
    
    // Get total frame size
    static constexpr size_t frameSize() {
        return 2 + 2 + 4 + 2 + DMX_UNIVERSE_SIZE + 2; // 524 bytes
    }
    
    // Get CRC calculation start offset (after preamble)
    static constexpr size_t crcOffset() {
        return 2;  // Start after preamble
    }
    
    // Get CRC calculation length (universe through payload)
    static constexpr size_t crcLength() {
        return 2 + 4 + 2 + DMX_UNIVERSE_SIZE;  // 520 bytes
    }
};

#pragma pack(pop)

static_assert(sizeof(SerialDmxFrame) == 524, "SerialDmxFrame size");

// ═══════════════════════════════════════════════════════════════════
// NODE HEARTBEAT PACKET (Optional)
// ═══════════════════════════════════════════════════════════════════
//
// Nodes can optionally send heartbeat stats back to gateway.
// This is a lightweight status update.
//
// ═══════════════════════════════════════════════════════════════════

#pragma pack(push, 1)

struct NodeHeartbeat {
    uint8_t  magic[2];      // 'A', 'H' for heartbeat
    uint8_t  version;
    uint8_t  role;          // Always ROLE_NODE
    uint16_t universe;      // Universe this node handles
    uint32_t last_seq;      // Last received sequence number
    uint32_t packets_recv;  // Total packets received
    uint32_t packets_drop;  // Dropped/invalid packets
    uint32_t frames_out;    // DMX frames output
    int8_t   rssi;          // Last received RSSI
    uint8_t  flags;         // Status flags
    uint16_t crc16;
    
    // Flag bits
    static constexpr uint8_t FLAG_DMX_ACTIVE = 0x01;
    static constexpr uint8_t FLAG_SIGNAL_OK  = 0x02;
    static constexpr uint8_t FLAG_OLED_OK    = 0x04;
    
    void init() {
        magic[0] = PACKET_MAGIC_0;
        magic[1] = 'H';
        version = PACKET_VERSION;
        role = ROLE_NODE;
        universe = 0;
        last_seq = 0;
        packets_recv = 0;
        packets_drop = 0;
        frames_out = 0;
        rssi = 0;
        flags = 0;
        crc16 = 0;
    }
};

#pragma pack(pop)

// ═══════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

// Calculate which chunk a DMX channel falls into
inline uint8_t channelToChunk(uint16_t channel) {
    if (channel == 0 || channel > DMX_UNIVERSE_SIZE) return 0xFF;
    return (channel - 1) / CHUNK_DATA_SIZE;
}

// Calculate offset within chunk for a DMX channel
inline uint16_t channelToChunkOffset(uint16_t channel) {
    if (channel == 0 || channel > DMX_UNIVERSE_SIZE) return 0xFFFF;
    return (channel - 1) % CHUNK_DATA_SIZE;
}

// Calculate data length for a given chunk index
inline uint16_t chunkDataLength(uint8_t chunk_index) {
    if (chunk_index < CHUNKS_PER_FRAME - 1) {
        return CHUNK_DATA_SIZE;
    }
    // Last chunk may be shorter
    return DMX_UNIVERSE_SIZE - (chunk_index * CHUNK_DATA_SIZE);
}

// Debug: Print packet info
inline void printPacketInfo(const DmxNowPacket& pkt) {
    Serial.printf("  Magic: %c%c Version: %d Role: %d\n",
        pkt.magic[0], pkt.magic[1], pkt.version, pkt.role);
    Serial.printf("  Universe: %d Seq: %lu Chunk: %d/%d Len: %d\n",
        pkt.universe, pkt.seq, pkt.chunk_index + 1, 
        pkt.chunk_total, pkt.data_len);
}

#endif // AETHER_ESPNOW_PACKET_H
