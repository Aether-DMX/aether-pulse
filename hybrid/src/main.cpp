/**
 * AETHER Pulse - Pure sACN/E1.31 DMX Node v2.2.0
 *
 * Clean, professional wireless DMX receiver using industry-standard sACN.
 * No ESP-NOW, no complex transport layers - just sACN multicast over WiFi.
 *
 * Features:
 * - sACN/E1.31 multicast reception (works with OLA, QLC+, any sACN source)
 * - Channel slice output (multiple nodes can share a universe)
 * - UDP JSON for configuration commands only (not DMX data)
 * - RS485/DMX output at configurable fixed rate (default 40Hz)
 * - Hold-last-frame on signal loss with fade to black
 * - Multi-sender detection with last-wins policy
 * - Periodic status logging for field debugging
 * - OLED status display
 * - OTA firmware updates
 * - NVS configuration storage
 *
 * Build: pio run
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncE131.h>
#include <esp_dmx.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════════════════
// VERSION
// ═══════════════════════════════════════════════════════════════════
#define FIRMWARE_VERSION "2.2.0"

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════
#define DMX_TX_PIN 17
#define DMX_RX_PIN 16
#define DMX_ENABLE_PIN 4
#define DMX_PORT 1

#define LED_PIN 2
#define OLED_SDA 21
#define OLED_SCL 22

// ═══════════════════════════════════════════════════════════════════
// NETWORK CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
const char* WIFI_SSID = "AetherDMX";
const char* WIFI_PASSWORD = "";  // Open network

const int CONFIG_PORT = 8888;         // UDP port for JSON config commands
const int DISCOVERY_PORT = 9999;      // UDP port for discovery/heartbeat
const char* CONTROLLER_IP = "192.168.50.1";  // Pi's IP on AetherDMX network

// ═══════════════════════════════════════════════════════════════════
// TIMING CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
#define DMX_PACKET_SIZE 513
#define DMX_OUTPUT_FPS 40               // Fixed output rate (25ms interval)
#define DMX_OUTPUT_INTERVAL_MS (1000 / DMX_OUTPUT_FPS)
#define SACN_TIMEOUT_MS 5000            // Hold last frame for 5s, then fade
#define SACN_STALE_LOG_INTERVAL_MS 2000 // Rate-limit stale universe logs
#define STATUS_LOG_INTERVAL_MS 10000    // Status report every 10s

// ═══════════════════════════════════════════════════════════════════
// SLICE MODE ENUM
// ═══════════════════════════════════════════════════════════════════
enum SliceMode {
    SLICE_ZERO_OUTSIDE = 0,      // Channels outside slice are forced to 0 (default)
    SLICE_PASS_THROUGH = 1       // Channels outside slice pass through from input
};

// ═══════════════════════════════════════════════════════════════════
// OLED DISPLAY
// ═══════════════════════════════════════════════════════════════════
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledPresent = false;

// ═══════════════════════════════════════════════════════════════════
// sACN SENDER TRACKING (multi-sender policy: last-wins)
// ═══════════════════════════════════════════════════════════════════
struct SenderInfo {
    IPAddress ip;
    uint8_t cid[16];          // sACN CID (Component Identifier)
    unsigned long lastSeen;
    uint32_t packetCount;
    bool valid;
};

SenderInfo currentSender;
unsigned long lastSenderChangeLog = 0;

// ═══════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════
Preferences preferences;
WiFiUDP configUdp;
WiFiUDP discoveryUdp;
ESPAsyncE131 e131(1);  // Single universe buffer

// DMX buffers - decoupled receive from output with slice assembly
uint8_t dmxIn[DMX_PACKET_SIZE] = {0};   // Incoming sACN data (full 512 channels)
uint8_t dmxOut[DMX_PACKET_SIZE] = {0};  // Output frame (slice-assembled)
volatile bool dmxDirty = false;          // Flag for new data received

String nodeId = "";
String nodeName = "";

// Configuration (stored in NVS)
int sourceUniverse = 1;       // sACN universe to listen on (1-63999)
int sliceStart = 1;           // First channel of slice (1-512)
int sliceEnd = 512;           // Last channel of slice (1-512)
SliceMode sliceMode = SLICE_ZERO_OUTSIDE;  // How to handle channels outside slice

// Stats - counters for observability
uint32_t sacnPacketsReceived = 0;
uint32_t sacnPacketsIgnored = 0;  // Packets for wrong universe (should be 0)
uint32_t dmxFramesSent = 0;
uint32_t configCommandsReceived = 0;
uint32_t senderChanges = 0;

// Timing - all independent
unsigned long lastSacnReceived = 0;
unsigned long lastDmxSend = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastStatusLog = 0;

// ═══════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════
void initWiFi();
void initSacn();
void initDmxOutput();
void initOLED();
void initOTA();
void assembleOutputFrame();
void outputDmxFrame();
void updateOLED();
void handleConfigCommand(const String& json);
void sendHeartbeat();
void sendRegistration();
void loadConfig();
void saveConfig();
void logStatus();
void processSacnPacket(e131_packet_t* packet);

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION STORAGE
// ═══════════════════════════════════════════════════════════════════
void loadConfig() {
    preferences.begin("aether", true);
    sourceUniverse = preferences.getInt("universe", 1);
    sliceStart = preferences.getInt("slice_start", 1);
    sliceEnd = preferences.getInt("slice_end", 512);
    sliceMode = (SliceMode)preferences.getInt("slice_mode", SLICE_ZERO_OUTSIDE);
    nodeName = preferences.getString("name", "");
    preferences.end();

    // Validate universe range (sACN allows 1-63999)
    if (sourceUniverse < 1 || sourceUniverse > 63999) {
        Serial.printf("⚠️ Invalid universe %d, resetting to 1\n", sourceUniverse);
        sourceUniverse = 1;
    }

    // Validate slice range
    if (sliceStart < 1) sliceStart = 1;
    if (sliceStart > 512) sliceStart = 512;
    if (sliceEnd < 1) sliceEnd = 1;
    if (sliceEnd > 512) sliceEnd = 512;
    if (sliceEnd < sliceStart) sliceEnd = sliceStart;

    // Validate slice mode
    if (sliceMode != SLICE_ZERO_OUTSIDE && sliceMode != SLICE_PASS_THROUGH) {
        sliceMode = SLICE_ZERO_OUTSIDE;
    }

    // Generate default name from MAC if not set
    if (nodeName.length() == 0) {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macStr[5];
        sprintf(macStr, "%02X%02X", mac[4], mac[5]);
        nodeName = String("PULSE-") + String(macStr);
    }

    Serial.printf("Config: Universe=%d, Slice=%d-%d, Mode=%s, Name=%s\n",
                  sourceUniverse, sliceStart, sliceEnd,
                  sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
                  nodeName.c_str());
}

void saveConfig() {
    preferences.begin("aether", false);
    preferences.putInt("universe", sourceUniverse);
    preferences.putInt("slice_start", sliceStart);
    preferences.putInt("slice_end", sliceEnd);
    preferences.putInt("slice_mode", (int)sliceMode);
    preferences.putString("name", nodeName);
    preferences.end();
    Serial.println("Config saved to NVS");
}

// ═══════════════════════════════════════════════════════════════════
// WIFI
// ═══════════════════════════════════════════════════════════════════
void initWiFi() {
    Serial.printf("Connecting to %s...\n", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    if (strlen(WIFI_PASSWORD) > 0) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    } else {
        WiFi.begin(WIFI_SSID);
    }

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        digitalWrite(LED_PIN, attempts % 2);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    } else {
        Serial.println("\nWiFi connection failed! Will retry...");
    }
}

// ═══════════════════════════════════════════════════════════════════
// sACN/E1.31 RECEPTION
// ═══════════════════════════════════════════════════════════════════
void initSacn() {
    // Universe parameter is used correctly - no hardcoding to 1
    if (e131.begin(E131_MULTICAST, sourceUniverse, 1)) {
        Serial.printf("sACN: Listening on Universe %d (multicast 239.255.0.%d)\n",
                      sourceUniverse, sourceUniverse);
    } else {
        Serial.println("sACN: Init FAILED!");
    }

    // Reset sender tracking for new universe
    currentSender.valid = false;
    currentSender.packetCount = 0;
}

void changeUniverse(int newUniverse) {
    // Validate universe range (1-63999 per sACN spec)
    if (newUniverse < 1 || newUniverse > 63999) {
        Serial.printf("⚠️ Invalid universe %d (must be 1-63999)\n", newUniverse);
        return;
    }
    if (newUniverse == sourceUniverse) return;

    Serial.printf("Universe: %d -> %d\n", sourceUniverse, newUniverse);
    sourceUniverse = newUniverse;

    // Reinitialize sACN with new universe
    initSacn();
    saveConfig();
}

void processSacnPacket(e131_packet_t* packet) {
    unsigned long now = millis();

    // Check for sender change (multi-sender detection)
    bool senderChanged = false;
    if (!currentSender.valid) {
        // First sender
        senderChanged = true;
        currentSender.valid = true;
        memcpy(currentSender.cid, &packet->cid, 16);
        currentSender.packetCount = 0;
        Serial.printf("sACN: First sender detected for Universe %d\n", sourceUniverse);
    } else {
        // Check if CID changed
        if (memcmp(currentSender.cid, &packet->cid, 16) != 0) {
            senderChanged = true;
            senderChanges++;

            // Rate-limit sender change logs
            if (now - lastSenderChangeLog > 5000) {
                char oldCid[9], newCid[9];
                snprintf(oldCid, sizeof(oldCid), "%02X%02X%02X%02X",
                    currentSender.cid[0], currentSender.cid[1],
                    currentSender.cid[2], currentSender.cid[3]);
                snprintf(newCid, sizeof(newCid), "%02X%02X%02X%02X",
                    packet->cid[0], packet->cid[1],
                    packet->cid[2], packet->cid[3]);
                Serial.printf("⚠️ Universe %d sender changed: %s -> %s (policy=last-wins)\n",
                    sourceUniverse, oldCid, newCid);
                lastSenderChangeLog = now;
            }

            memcpy(currentSender.cid, &packet->cid, 16);
        }
    }

    currentSender.lastSeen = now;
    currentSender.packetCount++;

    // Copy DMX data to input buffer (property_values[0] is start code, channels at 1-512)
    // This decouples receive from output - we just update the input buffer
    memcpy(dmxIn + 1, packet->property_values + 1, 512);
    dmxDirty = true;

    sacnPacketsReceived++;
    lastSacnReceived = now;
}

// ═══════════════════════════════════════════════════════════════════
// DMX OUTPUT (Fixed-rate, decoupled from receive, with slice assembly)
// ═══════════════════════════════════════════════════════════════════
void initDmxOutput() {
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    dmx_driver_install(DMX_PORT, &config, NULL, 0);
    dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_ENABLE_PIN);
    Serial.printf("DMX: Output initialized @ %d fps (UART1)\n", DMX_OUTPUT_FPS);
}

void assembleOutputFrame() {
    // Build dmxOut from dmxIn based on slice configuration
    // This is the key slice assembly logic

    if (sliceMode == SLICE_ZERO_OUTSIDE) {
        // Zero outside slice (default mode)
        // Only channels within slice are copied, rest are 0
        for (int ch = 1; ch <= 512; ch++) {
            if (ch >= sliceStart && ch <= sliceEnd) {
                dmxOut[ch] = dmxIn[ch];
            } else {
                dmxOut[ch] = 0;
            }
        }
    } else {
        // Pass-through mode: all channels from input
        memcpy(dmxOut + 1, dmxIn + 1, 512);
    }
}

void outputDmxFrame() {
    // Assemble output frame from input with slice logic
    assembleOutputFrame();

    // Output runs at fixed rate regardless of receive rate
    // This is the key timing separation: receive -> buffer -> assemble -> send
    dmx_write(DMX_PORT, dmxOut, DMX_PACKET_SIZE);
    dmx_send(DMX_PORT);
    dmx_wait_sent(DMX_PORT, DMX_TIMEOUT_TICK);
    dmxFramesSent++;
    dmxDirty = false;
}

// ═══════════════════════════════════════════════════════════════════
// STATUS LOGGING (Observability)
// ═══════════════════════════════════════════════════════════════════
void logStatus() {
    unsigned long now = millis();
    unsigned long uptimeSec = now / 1000;

    // Calculate actual output FPS
    static uint32_t lastFrameCount = 0;
    static unsigned long lastFpsCalc = 0;
    float actualFps = 0;

    if (now - lastFpsCalc >= 1000) {
        actualFps = (dmxFramesSent - lastFrameCount) * 1000.0 / (now - lastFpsCalc);
        lastFrameCount = dmxFramesSent;
        lastFpsCalc = now;
    }

    // Time since last sACN
    unsigned long timeSinceSacn = lastSacnReceived > 0 ? (now - lastSacnReceived) : 0;
    const char* sacnStatus = "NONE";
    if (lastSacnReceived > 0) {
        if (timeSinceSacn < 1000) sacnStatus = "LIVE";
        else if (timeSinceSacn < SACN_TIMEOUT_MS) sacnStatus = "STALE";
        else sacnStatus = "TIMEOUT";
    }

    // Sender info
    char senderStr[20] = "none";
    if (currentSender.valid) {
        snprintf(senderStr, sizeof(senderStr), "%02X%02X..%02X%02X",
            currentSender.cid[0], currentSender.cid[1],
            currentSender.cid[14], currentSender.cid[15]);
    }

    // Slice boundary preview
    uint8_t previewAtStart = (sliceStart >= 1 && sliceStart <= 512) ? dmxOut[sliceStart] : 0;
    uint8_t previewAtEnd = (sliceEnd >= 1 && sliceEnd <= 512) ? dmxOut[sliceEnd] : 0;
    uint8_t previewBeforeStart = (sliceStart > 1) ? dmxOut[sliceStart - 1] : 0;
    uint8_t previewAfterEnd = (sliceEnd < 512) ? dmxOut[sliceEnd + 1] : 0;

    Serial.println("───────────────────────────────────────────────");
    Serial.printf("STATUS @ %lus | %s | Universe %d\n", uptimeSec, nodeId.c_str(), sourceUniverse);
    Serial.println("───────────────────────────────────────────────");
    Serial.printf("  Slice:    %d-%d (%s)\n", sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through");
    Serial.printf("  sACN:     %s (last %lums ago)\n", sacnStatus, timeSinceSacn);
    Serial.printf("  Sender:   %s (changes: %lu)\n", senderStr, senderChanges);
    Serial.printf("  Packets:  RX=%lu, ignored=%lu\n", sacnPacketsReceived, sacnPacketsIgnored);
    Serial.printf("  Output:   TX=%lu, fps=%.1f (target %d)\n", dmxFramesSent, actualFps, DMX_OUTPUT_FPS);
    Serial.printf("  WiFi:     %s, RSSI=%d dBm\n",
        WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED", WiFi.RSSI());
    Serial.printf("  Preview:  ch1-4=[%d,%d,%d,%d]\n",
        dmxOut[1], dmxOut[2], dmxOut[3], dmxOut[4]);
    Serial.printf("  Boundary: [%d]@%d, [%d]@%d | [%d]@%d, [%d]@%d\n",
        previewBeforeStart, sliceStart - 1, previewAtStart, sliceStart,
        previewAtEnd, sliceEnd, previewAfterEnd, sliceEnd + 1);
    Serial.println("───────────────────────────────────────────────");
}

// ═══════════════════════════════════════════════════════════════════
// CONFIG COMMAND HANDLER (UDP JSON - NOT for DMX data)
// ═══════════════════════════════════════════════════════════════════
void handleConfigCommand(const String& jsonStr) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);

    if (error) {
        Serial.printf("JSON parse error: %s\n", error.c_str());
        return;
    }

    configCommandsReceived++;
    const char* cmd = doc["cmd"];
    if (!cmd) return;

    Serial.printf("Config cmd: %s\n", cmd);

    // Configuration update
    if (strcmp(cmd, "config") == 0) {
        bool configChanged = false;

        if (doc["name"].is<const char*>()) {
            nodeName = doc["name"].as<String>();
            configChanged = true;
        }
        if (doc["universe"].is<int>()) {
            changeUniverse(doc["universe"]);
            configChanged = true;
        }
        // Support both old and new field names for backward compatibility
        if (doc["slice_start"].is<int>()) {
            int newStart = doc["slice_start"];
            if (newStart >= 1 && newStart <= 512) {
                sliceStart = newStart;
                configChanged = true;
            }
        } else if (doc["channel_start"].is<int>()) {
            // Legacy field name support
            int newStart = doc["channel_start"];
            if (newStart >= 1 && newStart <= 512) {
                sliceStart = newStart;
                configChanged = true;
            }
        }
        if (doc["slice_end"].is<int>()) {
            int newEnd = doc["slice_end"];
            if (newEnd >= 1 && newEnd <= 512) {
                sliceEnd = newEnd;
                configChanged = true;
            }
        } else if (doc["channel_end"].is<int>()) {
            // Legacy field name support
            int newEnd = doc["channel_end"];
            if (newEnd >= 1 && newEnd <= 512) {
                sliceEnd = newEnd;
                configChanged = true;
            }
        }
        if (doc["slice_mode"].is<const char*>()) {
            const char* mode = doc["slice_mode"];
            if (strcmp(mode, "zero_outside") == 0) {
                sliceMode = SLICE_ZERO_OUTSIDE;
                configChanged = true;
            } else if (strcmp(mode, "pass_through") == 0) {
                sliceMode = SLICE_PASS_THROUGH;
                configChanged = true;
            }
        }

        // Validate slice range
        if (sliceEnd < sliceStart) sliceEnd = sliceStart;

        if (configChanged) {
            saveConfig();
            Serial.printf("Config updated: Slice=%d-%d, Mode=%s\n",
                sliceStart, sliceEnd,
                sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through");
        }
        sendRegistration();
    }
    // Identify (flash LED)
    else if (strcmp(cmd, "identify") == 0) {
        Serial.println("Identify requested - flashing LED");
        for (int i = 0; i < 20; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
    // Blackout
    else if (strcmp(cmd, "blackout") == 0) {
        Serial.println("Blackout requested");
        memset(dmxIn + 1, 0, 512);
    }
    // All channels to value
    else if (strcmp(cmd, "all") == 0) {
        int val = doc["val"] | 0;
        Serial.printf("All channels to %d\n", val);
        memset(dmxIn + 1, val, 512);
    }
    // Reboot
    else if (strcmp(cmd, "reboot") == 0) {
        Serial.println("Reboot requested");
        delay(500);
        ESP.restart();
    }
    // Status request
    else if (strcmp(cmd, "status") == 0) {
        logStatus();
        sendRegistration();
    }
    // Unpair - reset to defaults and clear saved config
    else if (strcmp(cmd, "unpair") == 0) {
        Serial.println("═══════════════════════════════════════════════════");
        Serial.println("  UNPAIR: Resetting to factory defaults");
        Serial.println("═══════════════════════════════════════════════════");

        // Reset to defaults
        sourceUniverse = 1;
        sliceStart = 1;
        sliceEnd = 512;
        sliceMode = SLICE_ZERO_OUTSIDE;
        nodeName = "";  // Will use PULSE-XXXX from MAC

        // Clear saved config from NVS
        preferences.begin("aether", false);
        preferences.clear();
        preferences.end();

        Serial.println("Config cleared. Node ready for re-pairing.");
        Serial.printf("Defaults: Universe=%d, Slice=%d-%d, Mode=zero_outside\n",
            sourceUniverse, sliceStart, sliceEnd);

        // Re-register with Pi to show as unpaired
        sendRegistration();
    }
}

// ═══════════════════════════════════════════════════════════════════
// NETWORK MESSAGES
// ═══════════════════════════════════════════════════════════════════
void sendRegistration() {
    if (WiFi.status() != WL_CONNECTED) return;

    // Build registration JSON with both new and legacy field names for compatibility
    char json[640];
    snprintf(json, sizeof(json),
        "{\"type\":\"register\",\"node_id\":\"%s\",\"hostname\":\"%s\","
        "\"mac\":\"%s\",\"ip\":\"%s\",\"universe\":%d,"
        "\"slice_start\":%d,\"slice_end\":%d,\"slice_mode\":\"%s\","
        "\"startChannel\":%d,\"channelCount\":%d,"
        "\"firmware\":\"pulse-sacn\",\"version\":\"%s\","
        "\"transport\":\"sACN\",\"rssi\":%d,\"uptime\":%lu,"
        "\"sender_policy\":\"last-wins\"}",
        nodeId.c_str(), nodeName.c_str(),
        WiFi.macAddress().c_str(), WiFi.localIP().toString().c_str(),
        sourceUniverse,
        sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
        sliceStart, sliceEnd - sliceStart + 1,  // Legacy fields for compatibility
        FIRMWARE_VERSION, WiFi.RSSI(), millis() / 1000);

    discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
    discoveryUdp.print(json);
    discoveryUdp.endPacket();
}

void sendHeartbeat() {
    if (WiFi.status() != WL_CONNECTED) return;

    // Include sender info in heartbeat for Pi-side observability
    char senderCid[17] = "none";
    if (currentSender.valid) {
        snprintf(senderCid, sizeof(senderCid), "%02X%02X%02X%02X%02X%02X%02X%02X",
            currentSender.cid[0], currentSender.cid[1],
            currentSender.cid[2], currentSender.cid[3],
            currentSender.cid[4], currentSender.cid[5],
            currentSender.cid[6], currentSender.cid[7]);
    }

    char json[512];
    snprintf(json, sizeof(json),
        "{\"type\":\"heartbeat\",\"node_id\":\"%s\",\"rssi\":%d,"
        "\"uptime\":%lu,\"sacn_pkts\":%lu,\"dmx_frames\":%lu,"
        "\"universe\":%d,\"slice_start\":%d,\"slice_end\":%d,"
        "\"slice_mode\":\"%s\",\"transport\":\"sACN\","
        "\"sender_cid\":\"%s\",\"sender_changes\":%lu}",
        nodeId.c_str(), WiFi.RSSI(), millis() / 1000,
        sacnPacketsReceived, dmxFramesSent, sourceUniverse,
        sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
        senderCid, senderChanges);

    discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
    discoveryUdp.print(json);
    discoveryUdp.endPacket();
}

// ═══════════════════════════════════════════════════════════════════
// OLED DISPLAY
// ═══════════════════════════════════════════════════════════════════
void initOLED() {
    Wire.begin(OLED_SDA, OLED_SCL);

    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledPresent = true;
        Serial.println("OLED: Found at 0x3C");
    } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        oledPresent = true;
        Serial.println("OLED: Found at 0x3D");
    } else {
        Serial.println("OLED: Not detected");
        return;
    }

    // Boot screen
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(20, 8);
    display.print("AETHER");
    display.setTextSize(1);
    display.setCursor(40, 30);
    display.print("PULSE");
    display.setCursor(25, 45);
    display.printf("v%s", FIRMWARE_VERSION);
    display.display();
    delay(1000);
}

void drawSignalBars(int x, int y, int rssi) {
    int bars = 0;
    if (rssi > -50) bars = 4;
    else if (rssi > -60) bars = 3;
    else if (rssi > -70) bars = 2;
    else if (rssi > -80) bars = 1;

    for (int i = 0; i < 4; i++) {
        int barHeight = 3 + (i * 2);
        int barY = y + (8 - barHeight);
        if (i < bars) {
            display.fillRect(x + (i * 4), barY, 3, barHeight, SSD1306_WHITE);
        } else {
            display.drawRect(x + (i * 4), barY, 3, barHeight, SSD1306_WHITE);
        }
    }
}

void updateOLED() {
    if (!oledPresent) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Header: Node name + signal
    display.setCursor(0, 0);
    String displayName = nodeName.substring(0, 12);
    display.print(displayName);

    if (WiFi.status() == WL_CONNECTED) {
        drawSignalBars(100, 0, WiFi.RSSI());
    } else {
        display.setCursor(100, 0);
        display.print("--");
    }

    // Separator
    display.drawLine(0, 12, 128, 12, SSD1306_WHITE);

    // Universe (large) - supports any universe, not just 1
    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print("U");
    display.print(sourceUniverse);

    // Mode
    display.setTextSize(1);
    display.setCursor(60, 18);
    display.print("sACN");

    // Slice info (shows the channel range this node outputs)
    display.setCursor(60, 28);
    display.printf("%d-%d", sliceStart, sliceEnd);

    // sACN status with sender indicator
    display.setCursor(0, 40);
    unsigned long timeSinceSacn = millis() - lastSacnReceived;
    if (lastSacnReceived == 0) {
        display.print("Waiting for sACN...");
    } else if (timeSinceSacn < 1000) {
        display.print("sACN: LIVE");
        // Activity indicator
        display.fillCircle(120, 43, 3, SSD1306_WHITE);
    } else if (timeSinceSacn < SACN_TIMEOUT_MS) {
        display.printf("sACN: %lus ago", timeSinceSacn / 1000);
    } else {
        display.print("sACN: TIMEOUT");
    }

    // Stats row with sender changes if any
    display.setCursor(0, 52);
    if (senderChanges > 0) {
        display.printf("RX:%lu S:%lu", sacnPacketsReceived % 100000, senderChanges);
    } else {
        display.printf("RX:%lu TX:%lu", sacnPacketsReceived % 100000, dmxFramesSent % 100000);
    }

    display.display();
}

// ═══════════════════════════════════════════════════════════════════
// OTA UPDATES
// ═══════════════════════════════════════════════════════════════════
void initOTA() {
    ArduinoOTA.setHostname(nodeName.c_str());

    ArduinoOTA.onStart([]() {
        Serial.println("OTA: Update starting...");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA: Complete! Rebooting...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA: %u%%\r", (progress / (total / 100)));
        digitalWrite(LED_PIN, (progress / 1000) % 2);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]\n", error);
    });

    ArduinoOTA.begin();
    Serial.printf("OTA: Ready at %s.local\n", nodeName.c_str());
}

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(100);

    // Initialize DMX buffers
    memset(dmxIn, 0, sizeof(dmxIn));
    memset(dmxOut, 0, sizeof(dmxOut));

    // Initialize sender tracking
    currentSender.valid = false;
    currentSender.packetCount = 0;
    senderChanges = 0;

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Generate node ID from MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    nodeId = String("pulse-") + String(macStr).substring(8);

    // Boot banner
    Serial.println();
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println("  AETHER Pulse - sACN/E1.31 DMX Node");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("  Firmware:  pulse-sacn v%s\n", FIRMWARE_VERSION);
    Serial.printf("  Node ID:   %s\n", nodeId.c_str());
    Serial.printf("  Transport: sACN/E1.31 multicast\n");
    Serial.printf("  Output:    %d fps (fixed rate)\n", DMX_OUTPUT_FPS);
    Serial.printf("  Features:  channel-slice, multi-sender=last-wins\n");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println();

    // Initialize OLED
    initOLED();

    // Load config from NVS
    loadConfig();

    // Connect to WiFi
    initWiFi();

    if (WiFi.status() == WL_CONNECTED) {
        // Start UDP listeners
        configUdp.begin(CONFIG_PORT);
        discoveryUdp.begin(DISCOVERY_PORT);
        Serial.printf("UDP: Config=%d, Discovery=%d\n", CONFIG_PORT, DISCOVERY_PORT);

        // Initialize sACN (universe from config, not hardcoded)
        initSacn();

        // Enable OTA
        initOTA();

        // Announce ourselves
        sendRegistration();
    }

    // Initialize DMX output
    initDmxOutput();

    Serial.println("\n═══════════════════════════════════════════════════");
    Serial.printf("  READY - Slice %d-%d on Universe %d\n", sliceStart, sliceEnd, sourceUniverse);
    Serial.println("═══════════════════════════════════════════════════\n");

    // Initial status log
    lastStatusLog = millis();

    digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
    unsigned long now = millis();

    // ─────────────────────────────────────────────────────────────────
    // sACN/E1.31 RECEPTION (updates dmxIn buffer, does NOT trigger output)
    // ─────────────────────────────────────────────────────────────────
    if (!e131.isEmpty()) {
        e131_packet_t packet;
        e131.pull(&packet);

        // The library already filters by universe, so this packet matches sourceUniverse
        processSacnPacket(&packet);
    }

    // Hold-last-frame timeout - fade to black after timeout with no sACN
    if (lastSacnReceived > 0 && (now - lastSacnReceived) > SACN_TIMEOUT_MS) {
        // Gradual fade to black (simple linear fade) - fade input buffer
        static unsigned long lastFadeStep = 0;
        if (now - lastFadeStep > 50) {  // Fade step every 50ms
            bool anyActive = false;
            for (int i = 1; i <= 512; i++) {
                if (dmxIn[i] > 0) {
                    dmxIn[i] = (dmxIn[i] > 5) ? dmxIn[i] - 5 : 0;
                    anyActive = true;
                }
            }
            lastFadeStep = now;
            if (!anyActive) {
                lastSacnReceived = 0;  // Reset so we stop fading
                Serial.println("sACN: Fade to black complete");
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // UDP CONFIG COMMANDS (not DMX data)
    // ─────────────────────────────────────────────────────────────────
    int packetSize = configUdp.parsePacket();
    if (packetSize > 0) {
        char buffer[512];
        int len = configUdp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';
            handleConfigCommand(String(buffer));
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // DMX OUTPUT @ FIXED RATE (decoupled from receive)
    // Slice assembly happens here: dmxIn -> slice logic -> dmxOut -> DMX
    // ─────────────────────────────────────────────────────────────────
    if (now - lastDmxSend >= DMX_OUTPUT_INTERVAL_MS) {
        outputDmxFrame();
        lastDmxSend = now;
    }

    // ─────────────────────────────────────────────────────────────────
    // OTA HANDLING
    // ─────────────────────────────────────────────────────────────────
    ArduinoOTA.handle();

    // ─────────────────────────────────────────────────────────────────
    // HEARTBEAT @ 10 seconds
    // ─────────────────────────────────────────────────────────────────
    if (now - lastHeartbeat >= 10000) {
        sendHeartbeat();
        lastHeartbeat = now;
    }

    // ─────────────────────────────────────────────────────────────────
    // WIFI RECONNECT CHECK @ 5 seconds
    // ─────────────────────────────────────────────────────────────────
    if (now - lastWifiCheck >= 5000) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost, reconnecting...");
            WiFi.reconnect();
        }
        lastWifiCheck = now;
    }

    // ─────────────────────────────────────────────────────────────────
    // OLED UPDATE @ 1 second
    // ─────────────────────────────────────────────────────────────────
    if (now - lastDisplayUpdate >= 1000) {
        updateOLED();
        lastDisplayUpdate = now;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Heartbeat blink
    }

    // ─────────────────────────────────────────────────────────────────
    // STATUS LOG @ 10 seconds (for field debugging)
    // ─────────────────────────────────────────────────────────────────
    if (now - lastStatusLog >= STATUS_LOG_INTERVAL_MS) {
        logStatus();
        lastStatusLog = now;
    }

    // Small yield to prevent watchdog
    yield();
}
