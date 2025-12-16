/**
 * AETHER ESP-NOW Node - ESP-NOW Receiver Implementation
 */

#include "espnow_rx.h"

// Static instance pointer for callback
EspNowReceiver* EspNowReceiver::instance_ = nullptr;

// Global instance
EspNowReceiver espNowRx;

EspNowReceiver::EspNowReceiver()
    : initialized_(false)
    , universe_filter_(DEFAULT_UNIVERSE)
    , channel_(ESPNOW_CHANNEL)
    , callback_(nullptr)
    , last_seq_(0) {
    stats_.reset();
    reassembly_.reset();
}

bool EspNowReceiver::begin(uint16_t universe) {
    instance_ = this;
    universe_filter_ = universe;
    
    // Set WiFi mode to Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Set the channel
    esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE);
    
    Serial.printf("[RX] WiFi MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("[RX] WiFi Channel: %d\n", channel_);
    Serial.printf("[RX] Universe filter: %d (0=all)\n", universe_filter_);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[RX] ERROR: ESP-NOW init failed!");
        return false;
    }
    
    // Register receive callback
    esp_now_register_recv_cb(onReceiveCallback);
    
    initialized_ = true;
    Serial.println("[RX] ESP-NOW receiver ready");
    
    return true;
}

void EspNowReceiver::onReceiveCallback(const esp_now_recv_info_t* info,
                                        const uint8_t* data, int len) {
    if (instance_) {
        // Get RSSI from the rx_ctrl structure if available
        int8_t rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
        instance_->handlePacket(data, len, rssi);
    }
}

void EspNowReceiver::process() {
    // Check for reassembly timeout
    checkTimeout();
}

void EspNowReceiver::setFrameCallback(FrameCallback callback) {
    callback_ = callback;
}

void EspNowReceiver::setUniverse(uint16_t universe) {
    universe_filter_ = universe;
    Serial.printf("[RX] Universe filter set to %d\n", universe);
}

bool EspNowReceiver::isReceiving() const {
    if (stats_.last_packet_time == 0) return false;
    return (millis() - stats_.last_packet_time) < TEST_MODE_TIMEOUT_MS;
}

uint32_t EspNowReceiver::timeSinceLastPacket() const {
    if (stats_.last_packet_time == 0) return UINT32_MAX;
    return millis() - stats_.last_packet_time;
}

void EspNowReceiver::handlePacket(const uint8_t* data, int len, int8_t rssi) {
    stats_.packets_received++;
    stats_.last_rssi = rssi;
    
    // Minimum packet size check
    if (len < (int)DmxNowPacket::headerSize() + 2) {  // +2 for CRC
        stats_.packets_invalid++;
        #if DEBUG_VERBOSE
        Serial.printf("[RX] Packet too short: %d bytes\n", len);
        #endif
        return;
    }
    
    // Cast to packet structure
    const DmxNowPacket* packet = (const DmxNowPacket*)data;
    
    // Validate magic and version
    if (!packet->isValid()) {
        stats_.packets_invalid++;
        #if DEBUG_VERBOSE
        Serial.printf("[RX] Invalid magic/version: %02X %02X v%d\n",
            packet->magic[0], packet->magic[1], packet->version);
        #endif
        return;
    }
    
    // Validate role (should be from gateway)
    if (packet->role != ROLE_GATEWAY) {
        stats_.packets_invalid++;
        #if DEBUG_VERBOSE
        Serial.printf("[RX] Wrong role: %d\n", packet->role);
        #endif
        return;
    }
    
    // Verify CRC
    size_t crc_data_len = DmxNowPacket::headerSize() + packet->data_len;
    uint16_t calculated_crc = crc16_ccitt(data, crc_data_len);
    
    // CRC is stored after the data
    uint16_t received_crc = data[crc_data_len] | (data[crc_data_len + 1] << 8);
    
    if (calculated_crc != received_crc) {
        stats_.packets_invalid++;
        #if DEBUG_VERBOSE
        Serial.printf("[RX] CRC mismatch: calc=%04X recv=%04X\n", 
            calculated_crc, received_crc);
        #endif
        return;
    }
    
    // Universe filter
    if (universe_filter_ != 0 && packet->universe != universe_filter_) {
        // Silently ignore packets for other universes
        return;
    }
    
    // Packet is valid
    stats_.packets_valid++;
    stats_.last_packet_time = millis();
    
    #if DEBUG_VERBOSE
    Serial.printf("[RX] Packet OK: U%d seq=%lu chunk=%d/%d len=%d rssi=%d\n",
        packet->universe, packet->seq, packet->chunk_index + 1,
        packet->chunk_total, packet->data_len, rssi);
    #endif
    
    #if DEBUG_PACKET_HEX
    Serial.print("[RX] Data: ");
    for (int i = 0; i < min((int)packet->data_len, 16); i++) {
        Serial.printf("%02X ", packet->data[i]);
    }
    Serial.println("...");
    #endif
    
    // Process the chunk
    handleChunk(*packet);
}

void EspNowReceiver::handleChunk(const DmxNowPacket& packet) {
    unsigned long now = millis();
    
    // Check if this is a new frame or continuation
    if (!reassembly_.active || 
        reassembly_.seq != packet.seq || 
        reassembly_.universe != packet.universe) {
        
        // If we had an incomplete frame, count it as dropped
        if (reassembly_.active && !reassembly_.isComplete()) {
            stats_.frames_dropped++;
            #if DEBUG_VERBOSE
            Serial.printf("[RX] Dropped incomplete frame: U%d seq=%lu chunks=%02X\n",
                reassembly_.universe, reassembly_.seq, reassembly_.chunks_received);
            #endif
        }
        
        // Check for sequence jump
        if (reassembly_.active && packet.seq > last_seq_ + SEQ_WINDOW) {
            stats_.sequence_resets++;
        }
        
        // Start new frame
        reassembly_.reset();
        reassembly_.active = true;
        reassembly_.universe = packet.universe;
        reassembly_.seq = packet.seq;
        reassembly_.chunk_total = packet.chunk_total;
        reassembly_.start_time = now;
    }
    
    // Validate chunk index
    if (packet.chunk_index >= packet.chunk_total) {
        #if DEBUG_VERBOSE
        Serial.printf("[RX] Invalid chunk index: %d >= %d\n",
            packet.chunk_index, packet.chunk_total);
        #endif
        return;
    }
    
    // Check if already received this chunk
    if (reassembly_.chunks_received & (1 << packet.chunk_index)) {
        // Duplicate chunk, ignore
        return;
    }
    
    // Copy chunk data to appropriate position
    uint16_t offset = packet.chunk_index * CHUNK_DATA_SIZE;
    uint16_t copy_len = min((uint16_t)packet.data_len, 
                           (uint16_t)(DMX_UNIVERSE_SIZE - offset));
    
    memcpy(reassembly_.data + offset, packet.data, copy_len);
    
    // Mark chunk as received
    reassembly_.chunks_received |= (1 << packet.chunk_index);
    
    // Check if frame is complete
    if (reassembly_.isComplete()) {
        frameComplete();
    }
}

void EspNowReceiver::frameComplete() {
    stats_.frames_complete++;
    last_seq_ = reassembly_.seq;
    
    #if DEBUG_VERBOSE
    Serial.printf("[RX] Frame complete: U%d seq=%lu\n",
        reassembly_.universe, reassembly_.seq);
    #endif
    
    // Invoke callback
    if (callback_) {
        callback_(reassembly_.universe, reassembly_.seq, reassembly_.data);
    }
    
    // Reset for next frame
    reassembly_.reset();
}

void EspNowReceiver::checkTimeout() {
    unsigned long now = millis();
    
    if (reassembly_.hasTimedOut(now)) {
        stats_.frames_dropped++;
        
        #if DEBUG_VERBOSE
        Serial.printf("[RX] Frame timeout: U%d seq=%lu chunks=%02X\n",
            reassembly_.universe, reassembly_.seq, reassembly_.chunks_received);
        #endif
        
        reassembly_.reset();
    }
}
