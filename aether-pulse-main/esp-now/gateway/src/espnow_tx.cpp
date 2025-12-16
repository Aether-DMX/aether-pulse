/**
 * AETHER ESP-NOW Gateway - ESP-NOW Transmitter Implementation
 */

#include "espnow_tx.h"

// Static instance pointer for callback
EspNowTransmitter* EspNowTransmitter::instance_ = nullptr;

// Global instance
EspNowTransmitter espNowTx;

EspNowTransmitter::EspNowTransmitter()
    : initialized_(false)
    , channel_(ESPNOW_CHANNEL) {
    stats_.reset();
    packet_.init();
}

bool EspNowTransmitter::begin() {
    instance_ = this;
    
    // Set WiFi mode to Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Set the channel
    esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE);
    
    Serial.printf("[TX] WiFi MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("[TX] WiFi Channel: %d\n", channel_);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[TX] ERROR: ESP-NOW init failed!");
        return false;
    }
    
    // Register send callback
    esp_now_register_send_cb(onSendCallback);
    
    // Add broadcast peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, ESPNOW_BROADCAST_ADDR, 6);
    peerInfo.channel = channel_;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[TX] ERROR: Failed to add broadcast peer!");
        return false;
    }
    
    initialized_ = true;
    Serial.println("[TX] ESP-NOW transmitter ready");
    
    return true;
}

void EspNowTransmitter::onSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (instance_) {
        instance_->handleSendResult(status);
    }
}

void EspNowTransmitter::handleSendResult(esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        stats_.callback_errors++;
        
        #if DEBUG_VERBOSE
        Serial.println("[TX] Send callback reported failure");
        #endif
    }
}

bool EspNowTransmitter::sendFrame(uint16_t universe, uint32_t seq, const uint8_t* dmx_data) {
    if (!initialized_) {
        Serial.println("[TX] ERROR: Not initialized!");
        return false;
    }
    
    bool success = true;
    
    // Calculate number of chunks needed
    uint8_t num_chunks = CHUNKS_PER_FRAME;
    
    // Send each chunk
    for (uint8_t i = 0; i < num_chunks; i++) {
        uint16_t offset = i * CHUNK_DATA_SIZE;
        uint16_t len = chunkDataLength(i);
        
        // Make sure we don't read past the DMX data
        if (offset + len > DMX_UNIVERSE_SIZE) {
            len = DMX_UNIVERSE_SIZE - offset;
        }
        
        if (!sendChunk(universe, seq, i, dmx_data + offset, len)) {
            success = false;
        }
        
        // Small delay between chunks to avoid overwhelming receivers
        // This is ~50µs, negligible for DMX timing
        delayMicroseconds(50);
    }
    
    if (success) {
        stats_.frames_sent++;
        stats_.last_send_time = millis();
    }
    
    #if DEBUG_VERBOSE
    if (stats_.frames_sent % 100 == 0) {
        Serial.printf("[TX] Frames: %lu, Packets: %lu, Errors: %lu\n",
            stats_.frames_sent, stats_.packets_sent, stats_.send_failures);
    }
    #endif
    
    return success;
}

bool EspNowTransmitter::sendChunk(uint16_t universe, uint32_t seq,
                                   uint8_t chunk_index, const uint8_t* data, uint16_t data_len) {
    // Build packet
    packet_.init();
    packet_.role = ROLE_GATEWAY;
    packet_.universe = universe;
    packet_.seq = seq;
    packet_.chunk_index = chunk_index;
    packet_.chunk_total = CHUNKS_PER_FRAME;
    packet_.data_len = data_len;
    
    // Copy DMX data
    memcpy(packet_.data, data, data_len);
    
    // Calculate CRC over everything except the CRC field itself
    // CRC covers: magic through data (header + data)
    size_t crc_len = DmxNowPacket::headerSize() + data_len;
    packet_.crc16 = crc16_ccitt((const uint8_t*)&packet_, crc_len);
    
    // Calculate total packet size
    size_t packet_size = crc_len + 2;  // +2 for CRC
    
    #if DEBUG_PACKET_HEX
    Serial.printf("[TX] Chunk %d/%d: ", chunk_index + 1, CHUNKS_PER_FRAME);
    for (size_t i = 0; i < min(packet_size, (size_t)32); i++) {
        Serial.printf("%02X ", ((uint8_t*)&packet_)[i]);
    }
    Serial.println("...");
    #endif
    
    // Send via ESP-NOW
    esp_err_t result = esp_now_send(ESPNOW_BROADCAST_ADDR, 
                                     (const uint8_t*)&packet_, 
                                     packet_size);
    
    if (result == ESP_OK) {
        stats_.packets_sent++;
        return true;
    } else {
        stats_.send_failures++;
        
        #if DEBUG_VERBOSE
        Serial.printf("[TX] Send failed: %d\n", result);
        #endif
        
        return false;
    }
}
