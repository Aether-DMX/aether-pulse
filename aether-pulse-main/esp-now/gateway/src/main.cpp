/**
 * AETHER ESP-NOW DMX Gateway
 * 
 * Receives DMX frames from Raspberry Pi over serial and broadcasts
 * them to ESP-NOW nodes. Falls back to test pattern generation
 * when no serial data is received.
 * 
 * Part of the AETHER DMX ecosystem.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "../common/config.h"
#include "../common/packet.h"
#include "../common/crc16.h"

#include "serial_rx.h"
#include "espnow_tx.h"
#include "test_pattern.h"

// ═══════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════

SerialReceiver serialRx;

// DMX buffer for current frame
uint8_t dmxBuffer[DMX_UNIVERSE_SIZE];

// State
uint16_t currentUniverse = DEFAULT_UNIVERSE;
uint32_t txSequence = 0;
bool testModeActive = false;

// Timing
unsigned long lastDmxSend = 0;
unsigned long lastDebugPrint = 0;
unsigned long lastTestPatternSend = 0;

// Statistics
uint32_t framesSentFromSerial = 0;
uint32_t framesSentFromTest = 0;

// ═══════════════════════════════════════════════════════════════════
// SERIAL FRAME CALLBACK
// ═══════════════════════════════════════════════════════════════════

void onSerialFrame(uint16_t universe, uint32_t seq, 
                   const uint8_t* dmx_data, uint16_t length) {
    // Update state
    currentUniverse = universe;
    
    // Copy data to buffer
    memcpy(dmxBuffer, dmx_data, min((int)length, DMX_UNIVERSE_SIZE));
    
    // Send over ESP-NOW
    txSequence++;
    if (espNowTx.sendFrame(universe, txSequence, dmxBuffer)) {
        framesSentFromSerial++;
        
        #if DEBUG_VERBOSE
        Serial.printf("[GW] Serial frame -> ESP-NOW: U%d seq=%lu\n", universe, txSequence);
        #endif
    }
    
    // Clear test mode flag
    testModeActive = false;
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
    // Initialize serial for debug output
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n");
    Serial.println("═══════════════════════════════════════════════════════");
    Serial.println("  AETHER ESP-NOW DMX Gateway");
    Serial.printf("  Version: %s\n", AETHER_ESPNOW_VERSION_STRING);
    Serial.println("═══════════════════════════════════════════════════════");
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // Clear DMX buffer
    memset(dmxBuffer, 0, sizeof(dmxBuffer));
    
    // Initialize ESP-NOW transmitter
    Serial.println("\n[GW] Initializing ESP-NOW...");
    if (!espNowTx.begin()) {
        Serial.println("[GW] ERROR: ESP-NOW init failed! Halting.");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }
    
    // Initialize serial receiver
    // Use Serial2 for Pi communication (Serial is for debug)
    Serial.println("[GW] Initializing serial receiver...");
    Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1, 16, 17);  // RX=16, TX=17
    serialRx.begin(Serial2, SERIAL_BAUD_RATE);
    serialRx.setFrameCallback(onSerialFrame);
    
    // Initialize test pattern generator
    testPattern.setPattern(TEST_PATTERN_MODE);
    
    Serial.println("\n[GW] Gateway ready!");
    Serial.printf("[GW] WiFi Channel: %d\n", espNowTx.getChannel());
    Serial.printf("[GW] Serial baud: %lu\n", (unsigned long)SERIAL_BAUD_RATE);
    Serial.printf("[GW] Test mode enabled: %s\n", TEST_PATTERN_ENABLED ? "YES" : "NO");
    Serial.println("═══════════════════════════════════════════════════════\n");
    
    digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
    unsigned long now = millis();
    
    // Process incoming serial data
    serialRx.process();
    
    // Check if we should enter test mode
    #if TEST_PATTERN_ENABLED
    if (!serialRx.isReceiving()) {
        if (!testModeActive) {
            Serial.println("[GW] No serial data - entering test mode");
            testModeActive = true;
            testPattern.reset();
        }
        
        // Generate and send test pattern at DMX rate
        if ((now - lastTestPatternSend) >= (1000 / DMX_REFRESH_HZ)) {
            if (testPattern.update(dmxBuffer)) {
                txSequence++;
                if (espNowTx.sendFrame(currentUniverse, txSequence, dmxBuffer)) {
                    framesSentFromTest++;
                }
            }
            lastTestPatternSend = now;
        }
    }
    #endif
    
    // Blink LED to show activity
    static unsigned long lastBlink = 0;
    if (now - lastBlink >= 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = now;
    }
    
    // Debug output
    if ((now - lastDebugPrint) >= DEBUG_PRINT_INTERVAL_MS) {
        const auto& rxStats = serialRx.getStats();
        const auto& txStats = espNowTx.getStats();
        
        Serial.println("────────────────────────────────────────");
        Serial.printf("[GW] Mode: %s\n", testModeActive ? "TEST PATTERN" : "SERIAL");
        Serial.printf("[GW] Serial: frames=%lu crc_err=%lu sync_err=%lu\n",
            rxStats.frames_received, rxStats.crc_errors, rxStats.sync_errors);
        Serial.printf("[GW] ESP-NOW: frames=%lu packets=%lu errors=%lu\n",
            txStats.frames_sent, txStats.packets_sent, 
            txStats.send_failures + txStats.callback_errors);
        Serial.printf("[GW] Output: U%d seq=%lu\n", currentUniverse, txSequence);
        Serial.printf("[GW] DMX Ch1-8: [%d,%d,%d,%d,%d,%d,%d,%d]\n",
            dmxBuffer[0], dmxBuffer[1], dmxBuffer[2], dmxBuffer[3],
            dmxBuffer[4], dmxBuffer[5], dmxBuffer[6], dmxBuffer[7]);
        
        lastDebugPrint = now;
    }
    
    // Handle serial commands for configuration
    while (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "stats") {
            const auto& rxStats = serialRx.getStats();
            const auto& txStats = espNowTx.getStats();
            Serial.println("\n=== Statistics ===");
            Serial.printf("Serial frames: %lu\n", rxStats.frames_received);
            Serial.printf("Serial CRC errors: %lu\n", rxStats.crc_errors);
            Serial.printf("Serial sync errors: %lu\n", rxStats.sync_errors);
            Serial.printf("ESP-NOW frames: %lu\n", txStats.frames_sent);
            Serial.printf("ESP-NOW packets: %lu\n", txStats.packets_sent);
            Serial.printf("ESP-NOW failures: %lu\n", txStats.send_failures);
            Serial.printf("Test mode frames: %lu\n", framesSentFromTest);
        }
        else if (cmd == "reset") {
            serialRx.resetStats();
            espNowTx.resetStats();
            framesSentFromSerial = 0;
            framesSentFromTest = 0;
            Serial.println("Statistics reset");
        }
        else if (cmd.startsWith("universe ")) {
            currentUniverse = cmd.substring(9).toInt();
            Serial.printf("Universe set to %d\n", currentUniverse);
        }
        else if (cmd.startsWith("pattern ")) {
            int p = cmd.substring(8).toInt();
            testPattern.setPattern(p);
            Serial.printf("Test pattern set to %d\n", p);
        }
        else if (cmd == "help") {
            Serial.println("\n=== Commands ===");
            Serial.println("stats    - Show statistics");
            Serial.println("reset    - Reset statistics");
            Serial.println("universe N - Set universe to N");
            Serial.println("pattern N  - Set test pattern (0=sine, 1=chase, 2=full, 3=ramp)");
        }
    }
}
