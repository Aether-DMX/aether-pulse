/**
 * AETHER ESP-NOW Gateway - Serial Frame Receiver Implementation
 */

#include "serial_rx.h"

SerialReceiver::SerialReceiver()
    : serial_(nullptr)
    , state_(SerialRxState::SYNC_0)
    , callback_(nullptr)
    , buffer_index_(0)
    , last_byte_time_(0) {
    stats_.reset();
    frame_.init();
}

void SerialReceiver::begin(HardwareSerial& serial, uint32_t baud_rate) {
    serial_ = &serial;
    serial_->begin(baud_rate);
    
    // Clear any pending data
    while (serial_->available()) {
        serial_->read();
    }
    
    state_ = SerialRxState::SYNC_0;
    buffer_index_ = 0;
    
    Serial.printf("[RX] Serial receiver started at %lu baud\n", baud_rate);
}

void SerialReceiver::setFrameCallback(FrameCallback callback) {
    callback_ = callback;
}

void SerialReceiver::process() {
    if (!serial_) return;
    
    unsigned long now = millis();
    
    // Check for timeout if we're in the middle of receiving
    if (state_ != SerialRxState::SYNC_0 && 
        (now - last_byte_time_) > SERIAL_TIMEOUT_MS) {
        
        #if DEBUG_VERBOSE
        Serial.printf("[RX] Timeout in state %d, resyncing\n", (int)state_);
        #endif
        
        stats_.timeout_errors++;
        resync();
    }
    
    // Process all available bytes
    while (serial_->available()) {
        uint8_t byte = serial_->read();
        last_byte_time_ = now;
        stats_.bytes_received++;
        
        handleByte(byte);
    }
}

void SerialReceiver::handleByte(uint8_t byte) {
    switch (state_) {
        case SerialRxState::SYNC_0:
            if (byte == SERIAL_PREAMBLE_0) {
                frame_.preamble[0] = byte;
                state_ = SerialRxState::SYNC_1;
            }
            break;
            
        case SerialRxState::SYNC_1:
            if (byte == SERIAL_PREAMBLE_1) {
                frame_.preamble[1] = byte;
                buffer_index_ = 0;
                state_ = SerialRxState::HEADER;
            } else if (byte == SERIAL_PREAMBLE_0) {
                // Stay looking for 0x55
                frame_.preamble[0] = byte;
            } else {
                // Lost sync
                stats_.sync_errors++;
                state_ = SerialRxState::SYNC_0;
            }
            break;
            
        case SerialRxState::HEADER:
            // Header is: universe (2) + seq (4) + length (2) = 8 bytes
            {
                uint8_t* header_ptr = ((uint8_t*)&frame_.universe);
                header_ptr[buffer_index_++] = byte;
                
                if (buffer_index_ >= 8) {
                    // Header complete, validate length
                    if (frame_.length > DMX_UNIVERSE_SIZE) {
                        #if DEBUG_VERBOSE
                        Serial.printf("[RX] Invalid length %d, resyncing\n", frame_.length);
                        #endif
                        stats_.sync_errors++;
                        resync();
                    } else {
                        buffer_index_ = 0;
                        state_ = SerialRxState::PAYLOAD;
                    }
                }
            }
            break;
            
        case SerialRxState::PAYLOAD:
            frame_.payload[buffer_index_++] = byte;
            
            if (buffer_index_ >= frame_.length) {
                buffer_index_ = 0;
                state_ = SerialRxState::CRC_0;
            }
            break;
            
        case SerialRxState::CRC_0:
            frame_.crc16 = byte;  // Low byte
            state_ = SerialRxState::CRC_1;
            break;
            
        case SerialRxState::CRC_1:
            frame_.crc16 |= (byte << 8);  // High byte
            frameComplete();
            break;
    }
}

void SerialReceiver::frameComplete() {
    // Validate CRC
    if (validateCrc()) {
        stats_.frames_received++;
        stats_.last_frame_time = millis();
        
        #if DEBUG_VERBOSE
        Serial.printf("[RX] Frame OK: U%d seq=%lu len=%d\n", 
            frame_.universe, frame_.seq, frame_.length);
        #endif
        
        // Invoke callback
        if (callback_) {
            callback_(frame_.universe, frame_.seq, frame_.payload, frame_.length);
        }
    } else {
        stats_.crc_errors++;
        
        #if DEBUG_VERBOSE
        Serial.printf("[RX] CRC error: U%d seq=%lu\n", frame_.universe, frame_.seq);
        #endif
    }
    
    // Reset for next frame
    resync();
}

bool SerialReceiver::validateCrc() {
    // CRC is calculated over: universe + seq + length + payload
    // That's bytes 2 through (2 + 8 + length - 1) of the frame
    uint16_t calculated = crc16_ccitt(
        (const uint8_t*)&frame_.universe,
        8 + frame_.length  // 8 bytes header + payload
    );
    
    return calculated == frame_.crc16;
}

void SerialReceiver::resync() {
    state_ = SerialRxState::SYNC_0;
    buffer_index_ = 0;
}

bool SerialReceiver::isReceiving() const {
    if (stats_.last_frame_time == 0) return false;
    return (millis() - stats_.last_frame_time) < TEST_MODE_TIMEOUT_MS;
}

uint32_t SerialReceiver::timeSinceLastFrame() const {
    if (stats_.last_frame_time == 0) return UINT32_MAX;
    return millis() - stats_.last_frame_time;
}
