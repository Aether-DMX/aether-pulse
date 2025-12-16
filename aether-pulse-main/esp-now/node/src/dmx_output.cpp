/**
 * AETHER ESP-NOW Node - DMX Output Implementation
 */

#include "dmx_output.h"

// Global instance
DmxOutput dmxOutput;

DmxOutput::DmxOutput()
    : initialized_(false)
    , running_(false)
    , active_buffer_(buffer_a_)
    , pending_buffer_(buffer_b_)
    , swap_pending_(false)
    , dmx_task_(nullptr)
    , output_hz_(DMX_REFRESH_HZ)
    , fps_frame_count_(0)
    , fps_start_time_(0) {
    
    stats_.reset();
    
    // Initialize buffers with start code
    memset(buffer_a_, 0, DMX_PACKET_SIZE);
    memset(buffer_b_, 0, DMX_PACKET_SIZE);
    buffer_a_[0] = 0;  // DMX start code
    buffer_b_[0] = 0;
    
    // Initialize mutex
    buffer_mutex_ = portMUX_INITIALIZER_UNLOCKED;
}

bool DmxOutput::begin() {
    Serial.println("[DMX] Initializing DMX output...");
    
    // Configure DMX driver
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    
    // Install DMX driver
    if (dmx_driver_install(DMX_PORT, &config, NULL, 0) != ESP_OK) {
        Serial.println("[DMX] ERROR: Driver install failed!");
        return false;
    }
    
    // Set pins
    if (dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_ENABLE_PIN) != ESP_OK) {
        Serial.println("[DMX] ERROR: Pin configuration failed!");
        return false;
    }
    
    initialized_ = true;
    
    Serial.printf("[DMX] Output ready: TX=%d, RX=%d, EN=%d\n",
        DMX_TX_PIN, DMX_RX_PIN, DMX_ENABLE_PIN);
    
    return true;
}

void DmxOutput::updateBuffer(const uint8_t* dmx_data) {
    // Copy to pending buffer (channels 1-512 go to bytes 1-512)
    memcpy(pending_buffer_ + 1, dmx_data, DMX_UNIVERSE_SIZE);
    swap_pending_ = true;
}

void DmxOutput::swapBuffers() {
    if (!swap_pending_) return;
    
    portENTER_CRITICAL(&buffer_mutex_);
    
    // Swap buffer pointers
    uint8_t* temp = active_buffer_;
    active_buffer_ = pending_buffer_;
    pending_buffer_ = temp;
    swap_pending_ = false;
    
    portEXIT_CRITICAL(&buffer_mutex_);
    
    stats_.buffer_swaps++;
    stats_.last_swap_time = millis();
}

void DmxOutput::sendFrame() {
    if (!initialized_) return;
    
    // Check if buffer swap is pending
    if (swap_pending_) {
        swapBuffers();
    }
    
    // Write and send DMX
    dmx_write(DMX_PORT, active_buffer_, DMX_PACKET_SIZE);
    dmx_send(DMX_PORT);
    dmx_wait_sent(DMX_PORT, DMX_TIMEOUT_TICK);
    
    stats_.frames_sent++;
    stats_.last_frame_time = millis();
    
    // FPS calculation
    fps_frame_count_++;
    unsigned long now = millis();
    if (now - fps_start_time_ >= 1000) {
        stats_.actual_fps = (float)fps_frame_count_ * 1000.0f / (now - fps_start_time_);
        fps_frame_count_ = 0;
        fps_start_time_ = now;
    }
}

void DmxOutput::startAutoOutput(uint8_t hz) {
    if (running_) {
        stopAutoOutput();
    }
    
    output_hz_ = hz;
    
    // Create high-priority task on Core 1
    xTaskCreatePinnedToCore(
        dmxTaskFunction,    // Task function
        "DMX_Output",       // Name
        4096,               // Stack size
        this,               // Parameter
        configMAX_PRIORITIES - 1,  // Priority (highest)
        &dmx_task_,         // Task handle
        1                   // Core 1 (keep WiFi/ESP-NOW on Core 0)
    );
    
    running_ = true;
    Serial.printf("[DMX] Auto output started at %d Hz on Core 1\n", hz);
}

void DmxOutput::stopAutoOutput() {
    if (dmx_task_) {
        vTaskDelete(dmx_task_);
        dmx_task_ = nullptr;
    }
    running_ = false;
    Serial.println("[DMX] Auto output stopped");
}

void DmxOutput::dmxTaskFunction(void* param) {
    DmxOutput* self = (DmxOutput*)param;
    self->dmxTaskLoop();
}

void DmxOutput::dmxTaskLoop() {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / output_hz_);
    
    while (true) {
        sendFrame();
        
        // Wait for next period
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

uint8_t DmxOutput::getChannel(uint16_t channel) const {
    if (channel == 0 || channel > DMX_UNIVERSE_SIZE) return 0;
    return active_buffer_[channel];
}
