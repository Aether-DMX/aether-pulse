/**
 * AETHER ESP-NOW DMX Node
 * 
 * Receives DMX frames over ESP-NOW and outputs DMX512 signal.
 * Features hold-last-look on signal loss and optional OLED display.
 * 
 * Part of the AETHER DMX ecosystem.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#include "../common/config.h"
#include "../common/packet.h"
#include "../common/crc16.h"

#include "espnow_rx.h"
#include "dmx_output.h"
#include "frame_buffer.h"

// ═══════════════════════════════════════════════════════════════════
// OLED DISPLAY
// ═══════════════════════════════════════════════════════════════════

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledPresent = false;

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

Preferences preferences;
String nodeId = "";
String nodeName = "";
uint16_t configuredUniverse = DEFAULT_UNIVERSE;

// ═══════════════════════════════════════════════════════════════════
// TIMING
// ═══════════════════════════════════════════════════════════════════

unsigned long lastDisplayUpdate = 0;
unsigned long lastDebugPrint = 0;
unsigned long displayFrame = 0;

// ═══════════════════════════════════════════════════════════════════
// CALLBACKS
// ═══════════════════════════════════════════════════════════════════

void onDmxFrame(uint16_t universe, uint32_t seq, const uint8_t* dmx_data) {
    // Update frame buffer
    frameBuffer.updateFrame(dmx_data, universe, seq);
    
    // Update DMX output buffer
    dmxOutput.updateBuffer(dmx_data);
    
    #if DEBUG_VERBOSE
    if (seq % 100 == 0) {
        Serial.printf("[NODE] Frame: U%d seq=%lu Ch1-4=[%d,%d,%d,%d]\n",
            universe, seq, dmx_data[0], dmx_data[1], dmx_data[2], dmx_data[3]);
    }
    #endif
}

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

void loadConfig() {
    preferences.begin("aether-node", true);
    configuredUniverse = preferences.getInt("universe", DEFAULT_UNIVERSE);
    nodeName = preferences.getString("name", "");
    preferences.end();
    
    if (nodeName.length() == 0) {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macStr[5];
        sprintf(macStr, "%02X%02X", mac[4], mac[5]);
        nodeName = String("NODE-") + String(macStr);
    }
    
    Serial.printf("[NODE] Config: Universe=%d, Name=%s\n", 
        configuredUniverse, nodeName.c_str());
}

void saveConfig() {
    preferences.begin("aether-node", false);
    preferences.putInt("universe", configuredUniverse);
    preferences.putString("name", nodeName);
    preferences.end();
    Serial.println("[NODE] Config saved");
}

// ═══════════════════════════════════════════════════════════════════
// OLED DISPLAY
// ═══════════════════════════════════════════════════════════════════

void initOLED() {
    Wire.begin(OLED_SDA, OLED_SCL);
    
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        oledPresent = true;
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        
        // Boot animation
        display.setTextSize(2);
        display.setCursor(10, 8);
        display.print("AETHER");
        display.setTextSize(1);
        display.setCursor(30, 30);
        display.print("ESP-NOW NODE");
        display.display();
        delay(1000);
        
        Serial.println("[NODE] OLED initialized");
    } else {
        Serial.println("[NODE] OLED not found");
    }
}

void updateOLED() {
    if (!oledPresent) return;
    
    displayFrame++;
    display.clearDisplay();
    
    // Header line
    display.setTextSize(1);
    display.setCursor(0, 0);
    
    // Node name (truncated)
    String displayName = nodeName;
    if (displayName.length() > 12) {
        displayName = displayName.substring(0, 11) + ".";
    }
    display.print(displayName);
    
    // Signal indicator
    if (espNowRx.isReceiving()) {
        // Signal bars based on RSSI
        int rssi = espNowRx.getLastRssi();
        int bars = 0;
        if (rssi > -50) bars = 4;
        else if (rssi > -60) bars = 3;
        else if (rssi > -70) bars = 2;
        else if (rssi > -80) bars = 1;
        
        for (int i = 0; i < 4; i++) {
            int barHeight = 3 + (i * 2);
            int barY = 8 - barHeight;
            int barX = 100 + (i * 4);
            if (i < bars) {
                display.fillRect(barX, barY, 3, barHeight, SSD1306_WHITE);
            } else {
                display.drawRect(barX, barY, 3, barHeight, SSD1306_WHITE);
            }
        }
    } else {
        // No signal - X
        display.drawLine(104, 0, 112, 8, SSD1306_WHITE);
        display.drawLine(112, 0, 104, 8, SSD1306_WHITE);
    }
    
    // DMX activity dot
    if (dmxOutput.getStats().frames_sent > 0 && (displayFrame % 2 == 0)) {
        display.fillCircle(122, 4, 3, SSD1306_WHITE);
    } else {
        display.drawCircle(122, 4, 3, SSD1306_WHITE);
    }
    
    // Separator
    display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
    
    // Main content area
    if (!espNowRx.isReceiving() && !frameBuffer.hasValidData()) {
        // No signal, no data
        display.setTextSize(1);
        display.setCursor(20, 20);
        display.print("WAITING FOR");
        display.setCursor(30, 32);
        display.print("SIGNAL...");
        
        // Animated dots
        for (int i = 0; i < (displayFrame % 4); i++) {
            display.print(".");
        }
    } else {
        // Show universe and status
        display.setTextSize(2);
        display.setCursor(0, 16);
        display.print("U");
        display.print(configuredUniverse);
        
        // Status badge
        display.setTextSize(1);
        display.setCursor(60, 18);
        if (espNowRx.isReceiving()) {
            display.print("LIVE");
        } else {
            display.print("HOLD");
        }
        
        // Statistics
        const auto& rxStats = espNowRx.getStats();
        const auto& dmxStats = dmxOutput.getStats();
        
        display.setCursor(0, 34);
        display.printf("RX: %lu", rxStats.frames_complete);
        
        display.setCursor(64, 34);
        display.printf("TX: %lu", dmxStats.frames_sent);
        
        // Channel preview (first 8 channels)
        display.setCursor(0, 46);
        display.print("Ch:");
        for (int i = 0; i < 8; i++) {
            uint8_t val = dmxOutput.getChannel(i + 1);
            int barHeight = map(val, 0, 255, 0, 8);
            int barX = 24 + (i * 12);
            if (barHeight > 0) {
                display.fillRect(barX, 54 - barHeight, 8, barHeight, SSD1306_WHITE);
            }
        }
        
        // RSSI
        display.setCursor(0, 56);
        display.printf("RSSI:%d", espNowRx.getLastRssi());
        
        // FPS
        display.setCursor(70, 56);
        display.printf("%.1fHz", dmxStats.actual_fps);
    }
    
    display.display();
}

// ═══════════════════════════════════════════════════════════════════
// SERIAL COMMAND HANDLER
// ═══════════════════════════════════════════════════════════════════

void handleSerialCommand(const String& cmd) {
    if (cmd == "stats") {
        const auto& rxStats = espNowRx.getStats();
        const auto& dmxStats = dmxOutput.getStats();
        
        Serial.println("\n=== Statistics ===");
        Serial.printf("ESP-NOW packets: %lu\n", rxStats.packets_received);
        Serial.printf("ESP-NOW valid: %lu\n", rxStats.packets_valid);
        Serial.printf("ESP-NOW invalid: %lu\n", rxStats.packets_invalid);
        Serial.printf("Frames complete: %lu\n", rxStats.frames_complete);
        Serial.printf("Frames dropped: %lu\n", rxStats.frames_dropped);
        Serial.printf("DMX frames out: %lu\n", dmxStats.frames_sent);
        Serial.printf("DMX FPS: %.1f\n", dmxStats.actual_fps);
        Serial.printf("Last RSSI: %d\n", rxStats.last_rssi);
    }
    else if (cmd == "reset") {
        espNowRx.resetStats();
        dmxOutput.resetStats();
        Serial.println("Statistics reset");
    }
    else if (cmd.startsWith("universe ")) {
        configuredUniverse = cmd.substring(9).toInt();
        espNowRx.setUniverse(configuredUniverse);
        saveConfig();
        Serial.printf("Universe set to %d\n", configuredUniverse);
    }
    else if (cmd.startsWith("name ")) {
        nodeName = cmd.substring(5);
        saveConfig();
        Serial.printf("Name set to %s\n", nodeName.c_str());
    }
    else if (cmd == "reboot") {
        Serial.println("Rebooting...");
        delay(500);
        ESP.restart();
    }
    else if (cmd == "help") {
        Serial.println("\n=== Commands ===");
        Serial.println("stats       - Show statistics");
        Serial.println("reset       - Reset statistics");
        Serial.println("universe N  - Set universe to N");
        Serial.println("name NAME   - Set node name");
        Serial.println("reboot      - Restart node");
    }
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n");
    Serial.println("═══════════════════════════════════════════════════════");
    Serial.println("  AETHER ESP-NOW DMX Node");
    Serial.printf("  Version: %s\n", AETHER_ESPNOW_VERSION_STRING);
    Serial.println("═══════════════════════════════════════════════════════");
    
    // Generate node ID
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", 
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    nodeId = String("node-") + String(macStr).substring(8);
    Serial.printf("[NODE] ID: %s\n", nodeId.c_str());
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // Initialize OLED
    initOLED();
    
    // Load configuration
    loadConfig();
    
    // Initialize DMX output
    Serial.println("\n[NODE] Initializing DMX output...");
    if (!dmxOutput.begin()) {
        Serial.println("[NODE] ERROR: DMX init failed! Halting.");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }
    
    // Start DMX output task
    dmxOutput.startAutoOutput(DMX_REFRESH_HZ);
    
    // Initialize ESP-NOW receiver
    Serial.println("\n[NODE] Initializing ESP-NOW...");
    if (!espNowRx.begin(configuredUniverse)) {
        Serial.println("[NODE] ERROR: ESP-NOW init failed! Halting.");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }
    espNowRx.setFrameCallback(onDmxFrame);
    
    Serial.println("\n[NODE] Node ready!");
    Serial.printf("[NODE] WiFi Channel: %d\n", ESPNOW_CHANNEL);
    Serial.printf("[NODE] Universe: %d\n", configuredUniverse);
    Serial.printf("[NODE] DMX Rate: %d Hz\n", DMX_REFRESH_HZ);
    Serial.println("═══════════════════════════════════════════════════════\n");
    
    digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
    unsigned long now = millis();
    
    // Process ESP-NOW receiver (timeouts)
    espNowRx.process();
    
    // Process frame buffer (fade-to-black if configured)
    frameBuffer.process();
    
    // Blink LED to show activity
    static unsigned long lastBlink = 0;
    if (now - lastBlink >= 500) {
        digitalWrite(LED_PIN, espNowRx.isReceiving() ? LOW : !digitalRead(LED_PIN));
        lastBlink = now;
    }
    
    // Update OLED display
    if ((now - lastDisplayUpdate) >= 250) {
        updateOLED();
        lastDisplayUpdate = now;
    }
    
    // Debug output
    if ((now - lastDebugPrint) >= DEBUG_PRINT_INTERVAL_MS) {
        const auto& rxStats = espNowRx.getStats();
        const auto& dmxStats = dmxOutput.getStats();
        
        Serial.println("────────────────────────────────────────");
        Serial.printf("[NODE] Status: %s\n", 
            espNowRx.isReceiving() ? "RECEIVING" : "HOLD-LAST-LOOK");
        Serial.printf("[NODE] ESP-NOW: recv=%lu valid=%lu invalid=%lu frames=%lu\n",
            rxStats.packets_received, rxStats.packets_valid,
            rxStats.packets_invalid, rxStats.frames_complete);
        Serial.printf("[NODE] DMX: frames=%lu fps=%.1f\n",
            dmxStats.frames_sent, dmxStats.actual_fps);
        Serial.printf("[NODE] RSSI: %d dBm\n", rxStats.last_rssi);
        Serial.printf("[NODE] Ch1-8: [%d,%d,%d,%d,%d,%d,%d,%d]\n",
            dmxOutput.getChannel(1), dmxOutput.getChannel(2),
            dmxOutput.getChannel(3), dmxOutput.getChannel(4),
            dmxOutput.getChannel(5), dmxOutput.getChannel(6),
            dmxOutput.getChannel(7), dmxOutput.getChannel(8));
        
        lastDebugPrint = now;
    }
    
    // Handle serial commands
    static String serialBuffer = "";
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                handleSerialCommand(serialBuffer);
            }
            serialBuffer = "";
        } else {
            serialBuffer += c;
        }
    }
}
