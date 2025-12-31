/**
 * AETHER Pulse Gateway Firmware v2.2.0
 *
 * Wired UART gateway for direct Pi GPIO connection.
 * Receives DMX data via UART from Pi, outputs via RS485/DMX.
 *
 * Hardware:
 *   Pi GPIO 14 (TX) --> ESP32 RX (GPIO 3 / Serial)
 *   Pi GPIO 15 (RX) <-- ESP32 TX (GPIO 1 / Serial)
 *   Common Ground
 *
 * Protocol:
 *   Pi sends JSON commands over UART at 115200 baud
 *   Commands: dmx, config, status
 *
 * Features:
 *   - UART input from Pi GPIO (no WiFi needed)
 *   - Channel slice support (same as WiFi nodes)
 *   - RS485/DMX output @ 40Hz fixed rate
 *   - OLED status display
 *   - Timeout fade-to-black on signal loss
 */

#include <Arduino.h>
#include <esp_dmx.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════
#define DMX_TX_PIN 17
#define DMX_RX_PIN 16
#define DMX_EN_PIN 4
#define DMX_PORT 1

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C

// UART from Pi uses default Serial (GPIO 1/3)
#define PI_UART Serial
#define PI_UART_BAUD 115200

// ═══════════════════════════════════════════════════════════════════
// TIMING CONSTANTS
// ═══════════════════════════════════════════════════════════════════
#define DMX_OUTPUT_INTERVAL_MS 25      // 40 fps output
#define STATUS_LOG_INTERVAL_MS 10000   // Log every 10s
#define OLED_UPDATE_INTERVAL_MS 500    // OLED refresh
#define SIGNAL_TIMEOUT_MS 5000         // Fade after 5s no signal
#define FADE_STEP_INTERVAL_MS 50       // Fade speed
#define HEARTBEAT_INTERVAL_MS 10000    // Heartbeat to Pi

// ═══════════════════════════════════════════════════════════════════
// DMX PACKET SIZE
// ═══════════════════════════════════════════════════════════════════
#define DMX_PACKET_SIZE 513

// ═══════════════════════════════════════════════════════════════════
// SLICE MODE
// ═══════════════════════════════════════════════════════════════════
enum SliceMode {
    SLICE_ZERO_OUTSIDE = 0,
    SLICE_PASS_THROUGH = 1
};

// ═══════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Preferences preferences;

// Node identity
char nodeId[32] = "gateway-0000";
char nodeName[32] = "GATEWAY";

// Configuration
int sourceUniverse = 1;
int sliceStart = 1;
int sliceEnd = 512;
SliceMode sliceMode = SLICE_ZERO_OUTSIDE;

// DMX buffers
uint8_t dmxIn[DMX_PACKET_SIZE] = {0};   // Incoming from Pi
uint8_t dmxOut[DMX_PACKET_SIZE] = {0};  // Output frame (slice-assembled)

// Timing
unsigned long lastDmxOutput = 0;
unsigned long lastSignalTime = 0;
unsigned long lastStatusLog = 0;
unsigned long lastOledUpdate = 0;
unsigned long lastHeartbeat = 0;
unsigned long bootTime = 0;

// Statistics
unsigned long rxPackets = 0;
unsigned long txPackets = 0;
bool hasSignal = false;
bool isFading = false;

// UART buffer for incoming JSON
String uartBuffer = "";

// ═══════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════
void loadConfig();
void saveConfig();
void setupDMX();
void setupOLED();
void generateNodeId();
void processUartInput();
void handleCommand(const char* json);
void handleDmxCommand(JsonDocument& doc);
void handleConfigCommand(JsonDocument& doc);
void handleStatusCommand();
void assembleOutputFrame();
void outputDMX();
void updateOLED();
void logStatus();
void sendHeartbeat();
void fadeToBlack();

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
    bootTime = millis();

    // Initialize UART to Pi
    PI_UART.begin(PI_UART_BAUD);
    PI_UART.setTimeout(10);

    // Generate node ID from MAC
    generateNodeId();

    // Load configuration from NVS
    loadConfig();

    // Initialize hardware
    setupOLED();
    setupDMX();

    // Print boot banner
    Serial.println();
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println("  AETHER Pulse Gateway - Wired DMX Node");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("  Firmware:  pulse-gateway v%d.%d.%d\n",
        AETHER_FIRMWARE_VERSION_MAJOR,
        AETHER_FIRMWARE_VERSION_MINOR,
        AETHER_FIRMWARE_VERSION_PATCH);
    Serial.printf("  Node ID:   %s\n", nodeId);
    Serial.println("  Transport: UART (Pi GPIO direct)");
    Serial.println("  Output:    40 fps (fixed rate)");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println();

    const char* sliceModeStr = (sliceMode == SLICE_ZERO_OUTSIDE) ? "zero_outside" : "pass_through";
    Serial.printf("Config: Universe=%d, Slice=%d-%d (%s), Name=%s\n",
        sourceUniverse, sliceStart, sliceEnd, sliceModeStr, nodeName);
    Serial.println();
    Serial.println("UART: Listening for Pi commands @ 115200 baud");
    Serial.println("DMX: Output initialized @ 40 fps (UART1)");
    Serial.println();
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
    unsigned long now = millis();

    // Process incoming UART data from Pi
    processUartInput();

    // Check for signal timeout
    if (hasSignal && (now - lastSignalTime > SIGNAL_TIMEOUT_MS)) {
        if (!isFading) {
            Serial.println("UART: Signal timeout - fading to black");
            isFading = true;
        }
        fadeToBlack();
    }

    // Fixed-rate DMX output @ 40 fps
    if (now - lastDmxOutput >= DMX_OUTPUT_INTERVAL_MS) {
        lastDmxOutput = now;
        assembleOutputFrame();
        outputDMX();
    }

    // Periodic status log
    if (now - lastStatusLog >= STATUS_LOG_INTERVAL_MS) {
        lastStatusLog = now;
        logStatus();
    }

    // OLED update
    if (now - lastOledUpdate >= OLED_UPDATE_INTERVAL_MS) {
        lastOledUpdate = now;
        updateOLED();
    }

    // Heartbeat to Pi
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        sendHeartbeat();
    }
}

// ═══════════════════════════════════════════════════════════════════
// UART INPUT PROCESSING
// ═══════════════════════════════════════════════════════════════════
void processUartInput() {
    while (PI_UART.available()) {
        char c = PI_UART.read();

        if (c == '\n' || c == '\r') {
            if (uartBuffer.length() > 0) {
                handleCommand(uartBuffer.c_str());
                uartBuffer = "";
            }
        } else {
            uartBuffer += c;
            // Prevent buffer overflow
            if (uartBuffer.length() > 2048) {
                uartBuffer = "";
            }
        }
    }
}

void handleCommand(const char* json) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
        Serial.printf("UART: JSON parse error: %s\n", error.c_str());
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
        return;
    }

    if (strcmp(cmd, "dmx") == 0) {
        handleDmxCommand(doc);
    } else if (strcmp(cmd, "config") == 0) {
        handleConfigCommand(doc);
    } else if (strcmp(cmd, "status") == 0) {
        handleStatusCommand();
    } else if (strcmp(cmd, "unpair") == 0) {
        // Reset to defaults
        sourceUniverse = 1;
        sliceStart = 1;
        sliceEnd = 512;
        sliceMode = SLICE_ZERO_OUTSIDE;
        strlcpy(nodeName, "GATEWAY", sizeof(nodeName));
        saveConfig();
        Serial.println("Config: Reset to defaults");
    }
}

void handleDmxCommand(JsonDocument& doc) {
    // Expected format: {"cmd":"dmx","universe":1,"data":[0,255,128,...]}
    int universe = doc["universe"] | 1;

    // Only process if universe matches
    if (universe != sourceUniverse) {
        return;
    }

    JsonArray data = doc["data"];
    if (data) {
        // Clear input buffer first
        memset(dmxIn + 1, 0, 512);

        int idx = 1;
        for (JsonVariant v : data) {
            if (idx > 512) break;
            dmxIn[idx++] = v.as<uint8_t>();
        }

        rxPackets++;
        lastSignalTime = millis();
        hasSignal = true;
        isFading = false;
    }
}

void handleConfigCommand(JsonDocument& doc) {
    bool changed = false;

    if (!doc["universe"].isNull()) {
        sourceUniverse = doc["universe"];
        changed = true;
    }
    if (!doc["name"].isNull()) {
        strlcpy(nodeName, doc["name"] | "GATEWAY", sizeof(nodeName));
        changed = true;
    }
    // Support both new and legacy field names
    if (!doc["channel_start"].isNull()) {
        sliceStart = doc["channel_start"];
        changed = true;
    } else if (!doc["slice_start"].isNull()) {
        sliceStart = doc["slice_start"];
        changed = true;
    }
    if (!doc["channel_end"].isNull()) {
        sliceEnd = doc["channel_end"];
        changed = true;
    } else if (!doc["slice_end"].isNull()) {
        sliceEnd = doc["slice_end"];
        changed = true;
    }
    if (!doc["slice_mode"].isNull()) {
        const char* mode = doc["slice_mode"];
        if (strcmp(mode, "pass_through") == 0) {
            sliceMode = SLICE_PASS_THROUGH;
        } else {
            sliceMode = SLICE_ZERO_OUTSIDE;
        }
        changed = true;
    }

    if (changed) {
        saveConfig();
        const char* sliceModeStr = (sliceMode == SLICE_ZERO_OUTSIDE) ? "zero_outside" : "pass_through";
        Serial.printf("Config: Universe=%d, Slice=%d-%d (%s), Name=%s\n",
            sourceUniverse, sliceStart, sliceEnd, sliceModeStr, nodeName);
    }
}

void handleStatusCommand() {
    // Send status back to Pi via UART
    char json[512];
    unsigned long uptime = (millis() - bootTime) / 1000;
    const char* sliceModeStr = (sliceMode == SLICE_ZERO_OUTSIDE) ? "zero_outside" : "pass_through";

    snprintf(json, sizeof(json),
        "{\"type\":\"status\",\"node_id\":\"%s\",\"name\":\"%s\","
        "\"universe\":%d,\"slice_start\":%d,\"slice_end\":%d,\"slice_mode\":\"%s\","
        "\"transport\":\"uart\",\"has_signal\":%s,\"rx_packets\":%lu,\"tx_packets\":%lu,"
        "\"uptime\":%lu,\"firmware\":\"pulse-gateway v%d.%d.%d\"}\n",
        nodeId, nodeName, sourceUniverse, sliceStart, sliceEnd, sliceModeStr,
        hasSignal ? "true" : "false", rxPackets, txPackets, uptime,
        AETHER_FIRMWARE_VERSION_MAJOR, AETHER_FIRMWARE_VERSION_MINOR, AETHER_FIRMWARE_VERSION_PATCH);

    PI_UART.print(json);
}

// ═══════════════════════════════════════════════════════════════════
// SLICE ASSEMBLY
// ═══════════════════════════════════════════════════════════════════
void assembleOutputFrame() {
    dmxOut[0] = 0; // DMX start code

    if (sliceMode == SLICE_PASS_THROUGH) {
        // Pass through all channels unchanged
        memcpy(dmxOut + 1, dmxIn + 1, 512);
    } else {
        // Zero outside slice
        for (int ch = 1; ch <= 512; ch++) {
            if (ch >= sliceStart && ch <= sliceEnd) {
                dmxOut[ch] = dmxIn[ch];
            } else {
                dmxOut[ch] = 0;
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
// DMX OUTPUT
// ═══════════════════════════════════════════════════════════════════
void setupDMX() {
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    dmx_driver_install(DMX_PORT, &config, NULL, 0);
    dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_EN_PIN);
    Serial.println("DMX: Driver installed on UART1");
}

void outputDMX() {
    dmx_write(DMX_PORT, dmxOut, DMX_PACKET_SIZE);
    dmx_send(DMX_PORT);
    dmx_wait_sent(DMX_PORT, DMX_TIMEOUT_TICK);
    txPackets++;
}

void fadeToBlack() {
    static unsigned long lastFadeStep = 0;
    unsigned long now = millis();

    if (now - lastFadeStep < FADE_STEP_INTERVAL_MS) {
        return;
    }
    lastFadeStep = now;

    bool allZero = true;
    for (int i = 1; i <= 512; i++) {
        if (dmxIn[i] > 0) {
            dmxIn[i] = (dmxIn[i] > 5) ? dmxIn[i] - 5 : 0;
            allZero = false;
        }
    }

    if (allZero) {
        isFading = false;
        hasSignal = false;
        Serial.println("UART: Fade to black complete");
    }
}

// ═══════════════════════════════════════════════════════════════════
// OLED DISPLAY
// ═══════════════════════════════════════════════════════════════════
void setupOLED() {
    Wire.begin(OLED_SDA, OLED_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED: Not found");
        return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("AETHER Gateway");
    display.println("Initializing...");
    display.display();
    Serial.println("OLED: Initialized");
}

void updateOLED() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Line 1: Node name
    display.setCursor(0, 0);
    display.printf("%s", nodeName);

    // Line 2: Transport + status
    display.setCursor(0, 12);
    display.printf("UART %s", hasSignal ? "LIVE" : "---");

    // Line 3: Universe + slice
    display.setCursor(0, 24);
    display.printf("U%d ch%d-%d", sourceUniverse, sliceStart, sliceEnd);

    // Line 4: Channel preview
    display.setCursor(0, 36);
    display.printf("DMX:%3d %3d %3d %3d",
        dmxOut[sliceStart],
        dmxOut[sliceStart + 1],
        dmxOut[sliceStart + 2],
        dmxOut[sliceStart + 3]);

    // Line 5: Stats
    display.setCursor(0, 48);
    display.printf("RX:%lu TX:%lu", rxPackets, txPackets);

    display.display();
}

// ═══════════════════════════════════════════════════════════════════
// STATUS LOGGING
// ═══════════════════════════════════════════════════════════════════
void logStatus() {
    unsigned long uptime = (millis() - bootTime) / 1000;
    unsigned long lastMs = hasSignal ? (millis() - lastSignalTime) : 0;
    const char* sliceModeStr = (sliceMode == SLICE_ZERO_OUTSIDE) ? "zero_outside" : "pass_through";

    Serial.println("───────────────────────────────────────────────");
    Serial.printf("STATUS @ %lus | %s | Universe %d\n", uptime, nodeId, sourceUniverse);
    Serial.println("───────────────────────────────────────────────");
    Serial.printf("  UART:     %s (last %lums ago)\n", hasSignal ? "LIVE" : "NONE", lastMs);
    Serial.printf("  Packets:  RX=%lu, TX=%lu\n", rxPackets, txPackets);
    Serial.printf("  Slice:    %d-%d (%s)\n", sliceStart, sliceEnd, sliceModeStr);
    Serial.printf("  Output:   fps=40.0 (target 40)\n");
    Serial.printf("  Channels: preview=[%d,%d,%d,%d] | boundary=ch%d:%d,ch%d:%d\n",
        dmxOut[sliceStart], dmxOut[sliceStart+1], dmxOut[sliceStart+2], dmxOut[sliceStart+3],
        sliceStart, dmxOut[sliceStart], sliceEnd, dmxOut[sliceEnd]);
    Serial.println("───────────────────────────────────────────────");
}

void sendHeartbeat() {
    char json[256];
    unsigned long uptime = (millis() - bootTime) / 1000;
    const char* sliceModeStr = (sliceMode == SLICE_ZERO_OUTSIDE) ? "zero_outside" : "pass_through";

    snprintf(json, sizeof(json),
        "{\"type\":\"heartbeat\",\"node_id\":\"%s\",\"name\":\"%s\","
        "\"universe\":%d,\"slice_start\":%d,\"slice_end\":%d,\"slice_mode\":\"%s\","
        "\"transport\":\"uart\",\"uptime\":%lu,\"firmware\":\"pulse-gateway v%d.%d.%d\"}\n",
        nodeId, nodeName, sourceUniverse, sliceStart, sliceEnd, sliceModeStr,
        uptime, AETHER_FIRMWARE_VERSION_MAJOR, AETHER_FIRMWARE_VERSION_MINOR, AETHER_FIRMWARE_VERSION_PATCH);

    PI_UART.print(json);
}

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION STORAGE
// ═══════════════════════════════════════════════════════════════════
void generateNodeId() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(nodeId, sizeof(nodeId), "gateway-%02X%02X%02X%02X",
        mac[2], mac[3], mac[4], mac[5]);
}

void loadConfig() {
    preferences.begin("aether", true);
    sourceUniverse = preferences.getInt("universe", 1);
    sliceStart = preferences.getInt("ch_start", 1);
    sliceEnd = preferences.getInt("ch_end", 512);
    sliceMode = (SliceMode)preferences.getInt("slice_mode", SLICE_ZERO_OUTSIDE);
    preferences.getString("name", nodeName, sizeof(nodeName));
    if (strlen(nodeName) == 0) {
        strlcpy(nodeName, "GATEWAY", sizeof(nodeName));
    }
    preferences.end();
}

void saveConfig() {
    preferences.begin("aether", false);
    preferences.putInt("universe", sourceUniverse);
    preferences.putInt("ch_start", sliceStart);
    preferences.putInt("ch_end", sliceEnd);
    preferences.putInt("slice_mode", sliceMode);
    preferences.putString("name", nodeName);
    preferences.end();
}
