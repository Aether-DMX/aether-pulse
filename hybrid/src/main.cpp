/**
 * AETHER Pulse - UDPJSON DMX Node v2.5.0
 *
 * Wireless DMX receiver using direct UDP JSON protocol.
 * Receives DMX commands from AETHER Core on port 6455.
 *
 * Features:
 * - UDPJSON DMX protocol (set, fade, blackout, ping/pong)
 * - Channel slice output (multiple nodes can share a universe)
 * - RS485/DMX output at configurable fixed rate (default 40Hz)
 * - Hold-last-frame on signal loss with fade to black
 * - Non-blocking fade engine
 * - Periodic status logging for field debugging
 * - OLED status display
 * - OTA firmware updates
 * - NVS configuration storage
 *
 * Protocol (port 6455):
 * - {"type":"set","universe":2,"channels":{"1":255,"2":128},"ts":...}
 * - {"type":"fade","universe":2,"duration_ms":1000,"channels":{"1":0},"ts":...}
 * - {"type":"blackout","universe":2,"ts":...}
 * - {"type":"ping","ts":...} -> responds with pong
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
#define FIRMWARE_VERSION "2.5.0"

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
const int UDPJSON_DMX_PORT = 6455;    // SSOT: Port for UDPJSON DMX commands
const char* CONTROLLER_IP = "192.168.50.1";  // Pi's IP on AetherDMX network

// ═══════════════════════════════════════════════════════════════════
// TIMING CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
#define DMX_PACKET_SIZE 513
#define DMX_OUTPUT_FPS 40               // Fixed output rate (25ms interval)
#define DMX_OUTPUT_INTERVAL_MS (1000 / DMX_OUTPUT_FPS)
#define SACN_TIMEOUT_MS 3000            // Switch to offline mode after 3s
#define SACN_STALE_LOG_INTERVAL_MS 2000 // Rate-limit stale universe logs
#define STATUS_LOG_INTERVAL_MS 10000    // Status report every 10s

// ═══════════════════════════════════════════════════════════════════
// OFFLINE PLAYBACK CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
#define LOOP_BUFFER_FRAMES 80           // ~2 seconds at 40fps
#define LOOP_BUFFER_CHANNELS 128        // Only buffer first 128 channels (memory constraint)
#define MAX_CHASE_STEPS 16              // Max steps in a stored chase
#define CHASE_STORAGE_KEY "chase_data"  // NVS key for chase storage

// ═══════════════════════════════════════════════════════════════════
// SLICE MODE ENUM
// ═══════════════════════════════════════════════════════════════════
enum SliceMode {
    SLICE_ZERO_OUTSIDE = 0,      // Channels outside slice are forced to 0 (default)
    SLICE_PASS_THROUGH = 1       // Channels outside slice pass through from input
};

// ═══════════════════════════════════════════════════════════════════
// OFFLINE PLAYBACK MODE
// ═══════════════════════════════════════════════════════════════════
enum OfflineMode {
    OFFLINE_NONE = 0,            // No offline playback - fade to black
    OFFLINE_LOOP = 1,            // Loop the last N seconds of DMX
    OFFLINE_CHASE = 2,           // Play stored chase definition
    OFFLINE_HOLD = 3             // Hold last frame indefinitely
};

// Chase step structure (stored in NVS)
struct ChaseStep {
    uint8_t channels[LOOP_BUFFER_CHANNELS];  // Channel values for this step
    uint16_t fadeMs;                          // Fade time to this step
    uint16_t holdMs;                          // Hold time at this step
};

// Chase definition
struct ChaseDefinition {
    uint8_t stepCount;                        // Number of steps (0 = no chase stored)
    uint8_t loopCount;                        // 0 = infinite loop
    ChaseStep steps[MAX_CHASE_STEPS];
};

// DMX Ring Buffer for loop playback
struct LoopBuffer {
    uint8_t frames[LOOP_BUFFER_FRAMES][LOOP_BUFFER_CHANNELS];
    uint8_t writeIndex;                       // Next frame to write
    uint8_t frameCount;                       // Frames currently in buffer
    uint8_t readIndex;                        // Current playback position
    bool recording;                           // Whether we're recording
    bool hasData;                             // Whether buffer has valid data
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
WiFiUDP udpjsonDmxUdp;  // Port 6455 for UDPJSON DMX commands
ESPAsyncE131 e131(1);  // Single universe buffer (legacy, kept for compatibility)

// ═══════════════════════════════════════════════════════════════════
// FADE ENGINE - Per-channel non-blocking fades
// ═══════════════════════════════════════════════════════════════════
struct ChannelFade {
    uint8_t startValue;
    uint8_t targetValue;
    unsigned long startTime;
    unsigned long durationMs;
    bool active;
};

ChannelFade channelFades[513];  // 1-indexed (channel 1 = index 1)
uint32_t udpjsonPacketsReceived = 0;  // Stats for UDPJSON

// DMX buffers - decoupled receive from output with slice assembly
uint8_t dmxIn[DMX_PACKET_SIZE] = {0};   // Incoming sACN data (full 512 channels)
uint8_t dmxOut[DMX_PACKET_SIZE] = {0};  // Output frame (slice-assembled)
volatile bool dmxDirty = false;          // Flag for new data received

String nodeId = "";
String nodeName = "";
String deviceLabel = "";  // Always shows PULSE-XXXX for hardware identification

// Configuration (stored in NVS)
bool isPaired = false;        // Whether node has been configured/paired
int sourceUniverse = 0;       // sACN universe to listen on (0 = not configured)
int sliceStart = 1;           // First channel of slice (1-512)
int sliceEnd = 512;           // Last channel of slice (1-512)
SliceMode sliceMode = SLICE_ZERO_OUTSIDE;  // How to handle channels outside slice

// Offline playback state
OfflineMode offlineMode = OFFLINE_NONE;     // Disabled for UDPJSON     // Default: loop last frames when offline
LoopBuffer loopBuffer;                       // Ring buffer for loop playback
ChaseDefinition storedChase;                 // Chase definition from NVS
bool isOffline = false;                      // Currently in offline playback mode
unsigned long offlineStartTime = 0;          // When offline mode started
uint8_t chaseCurrentStep = 0;                // Current step in chase playback
unsigned long chaseStepStartTime = 0;        // When current step started
bool chaseFading = false;                    // Currently fading between steps

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
void handleUdpjsonDmxCommand(const String& json, IPAddress senderIP);
void sendHeartbeat();
void sendRegistration();
void sendPong(IPAddress senderIP, int senderPort);
void loadConfig();
void saveConfig();
void logStatus();
void processSacnPacket(e131_packet_t* packet);
void tickFades();
void initFadeEngine();

// Offline playback functions
void initOfflinePlayback();
void recordFrame();
void startOfflineMode();
void stopOfflineMode();
void playOfflineFrame();
void loadStoredChase();
void saveStoredChase();
void handleChaseCommand(JsonDocument& doc);

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION STORAGE
// ═══════════════════════════════════════════════════════════════════
void loadConfig() {
    preferences.begin("aether", true);
    isPaired = preferences.getBool("is_paired", false);
    sourceUniverse = preferences.getInt("universe", 0);  // 0 = not configured
    sliceStart = preferences.getInt("slice_start", 1);
    sliceEnd = preferences.getInt("slice_end", 512);
    sliceMode = (SliceMode)preferences.getInt("slice_mode", SLICE_ZERO_OUTSIDE);
    nodeName = preferences.getString("name", "");
    preferences.end();

    // If not paired, universe should be 0 (waiting for config)
    if (!isPaired) {
        sourceUniverse = 0;
    }

    // Validate universe range (sACN allows 1-63999, 0 = not configured)
    if (sourceUniverse != 0 && (sourceUniverse < 1 || sourceUniverse > 63999)) {
        Serial.printf("⚠️ Invalid universe %d, resetting to unpaired\n", sourceUniverse);
        sourceUniverse = 0;
        isPaired = false;
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

    if (isPaired) {
        Serial.printf("Config: Universe=%d, Slice=%d-%d, Mode=%s, Name=%s\n",
                      sourceUniverse, sliceStart, sliceEnd,
                      sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
                      nodeName.c_str());
    } else {
        Serial.printf("Config: WAITING FOR PAIRING (Name=%s)\n", nodeName.c_str());
    }
}

void saveConfig() {
    preferences.begin("aether", false);
    preferences.putBool("is_paired", isPaired);
    preferences.putInt("universe", sourceUniverse);
    preferences.putInt("slice_start", sliceStart);
    preferences.putInt("slice_end", sliceEnd);
    preferences.putInt("slice_mode", (int)sliceMode);
    preferences.putString("name", nodeName);
    preferences.putInt("offline_mode", (int)offlineMode);
    preferences.end();
    Serial.println("Config saved to NVS");
}

// ═══════════════════════════════════════════════════════════════════
// OFFLINE PLAYBACK - Loop Buffer & Chase Storage
// ═══════════════════════════════════════════════════════════════════

void initOfflinePlayback() {
    // Initialize loop buffer
    memset(&loopBuffer, 0, sizeof(loopBuffer));
    loopBuffer.recording = true;
    loopBuffer.hasData = false;

    // Initialize chase storage
    memset(&storedChase, 0, sizeof(storedChase));

    // Load offline mode preference
    preferences.begin("aether", true);
    offlineMode = OFFLINE_NONE;  // Force OFFLINE_NONE for UDPJSON mode
    preferences.end();

    // Load stored chase from NVS
    loadStoredChase();

    Serial.printf("Offline playback: mode=%d, chase_steps=%d\n",
                  offlineMode, storedChase.stepCount);
}

void loadStoredChase() {
    preferences.begin("chase", true);

    storedChase.stepCount = preferences.getUChar("step_count", 0);
    storedChase.loopCount = preferences.getUChar("loop_count", 0);

    if (storedChase.stepCount > 0 && storedChase.stepCount <= MAX_CHASE_STEPS) {
        // Load each step
        for (int i = 0; i < storedChase.stepCount; i++) {
            char key[16];

            // Load fade/hold times
            snprintf(key, sizeof(key), "s%d_fade", i);
            storedChase.steps[i].fadeMs = preferences.getUShort(key, 500);

            snprintf(key, sizeof(key), "s%d_hold", i);
            storedChase.steps[i].holdMs = preferences.getUShort(key, 1000);

            // Load channel data (stored as blob)
            snprintf(key, sizeof(key), "s%d_ch", i);
            size_t len = preferences.getBytesLength(key);
            if (len > 0 && len <= LOOP_BUFFER_CHANNELS) {
                preferences.getBytes(key, storedChase.steps[i].channels, len);
            }
        }
        Serial.printf("Loaded chase: %d steps\n", storedChase.stepCount);
    }

    preferences.end();
}

void saveStoredChase() {
    preferences.begin("chase", false);

    preferences.putUChar("step_count", storedChase.stepCount);
    preferences.putUChar("loop_count", storedChase.loopCount);

    for (int i = 0; i < storedChase.stepCount && i < MAX_CHASE_STEPS; i++) {
        char key[16];

        snprintf(key, sizeof(key), "s%d_fade", i);
        preferences.putUShort(key, storedChase.steps[i].fadeMs);

        snprintf(key, sizeof(key), "s%d_hold", i);
        preferences.putUShort(key, storedChase.steps[i].holdMs);

        snprintf(key, sizeof(key), "s%d_ch", i);
        preferences.putBytes(key, storedChase.steps[i].channels, LOOP_BUFFER_CHANNELS);
    }

    preferences.end();
    Serial.printf("Saved chase: %d steps\n", storedChase.stepCount);
}

void recordFrame() {
    // Record current DMX input to loop buffer (only first N channels)
    if (!loopBuffer.recording) return;

    // Copy first LOOP_BUFFER_CHANNELS from dmxIn to buffer
    memcpy(loopBuffer.frames[loopBuffer.writeIndex], dmxIn + 1, LOOP_BUFFER_CHANNELS);

    // Advance write pointer
    loopBuffer.writeIndex = (loopBuffer.writeIndex + 1) % LOOP_BUFFER_FRAMES;

    // Track how many frames we have
    if (loopBuffer.frameCount < LOOP_BUFFER_FRAMES) {
        loopBuffer.frameCount++;
    }

    loopBuffer.hasData = true;
}

void startOfflineMode() {
    if (isOffline) return;

    isOffline = true;
    offlineStartTime = millis();
    loopBuffer.recording = false;  // Stop recording, start playback
    loopBuffer.readIndex = 0;      // Start from beginning of buffer

    // Reset chase state
    chaseCurrentStep = 0;
    chaseStepStartTime = millis();
    chaseFading = true;

    const char* modeStr = "UNKNOWN";
    switch (offlineMode) {
        case OFFLINE_NONE: modeStr = "FADE-TO-BLACK"; break;
        case OFFLINE_LOOP: modeStr = "LOOP"; break;
        case OFFLINE_CHASE: modeStr = "CHASE"; break;
        case OFFLINE_HOLD: modeStr = "HOLD"; break;
    }

    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("  OFFLINE MODE: %s\n", modeStr);
    if (offlineMode == OFFLINE_LOOP) {
        Serial.printf("  Buffer: %d frames (~%.1fs)\n",
                      loopBuffer.frameCount,
                      loopBuffer.frameCount / (float)DMX_OUTPUT_FPS);
    } else if (offlineMode == OFFLINE_CHASE && storedChase.stepCount > 0) {
        Serial.printf("  Chase: %d steps\n", storedChase.stepCount);
    }
    Serial.println("═══════════════════════════════════════════════════");
}

void stopOfflineMode() {
    if (!isOffline) return;

    isOffline = false;
    loopBuffer.recording = true;  // Resume recording

    unsigned long offlineDuration = millis() - offlineStartTime;
    Serial.printf("ONLINE: Back after %lums offline\n", offlineDuration);
}

void playOfflineFrame() {
    if (!isOffline) return;

    unsigned long now = millis();

    switch (offlineMode) {
        case OFFLINE_NONE:
            // Fade to black (already handled in main loop)
            break;

        case OFFLINE_HOLD:
            // Just keep last frame - do nothing, dmxIn already has it
            break;

        case OFFLINE_LOOP:
            if (loopBuffer.hasData && loopBuffer.frameCount > 0) {
                // Copy frame from buffer to dmxIn
                memcpy(dmxIn + 1, loopBuffer.frames[loopBuffer.readIndex], LOOP_BUFFER_CHANNELS);

                // Advance read pointer (loop around)
                loopBuffer.readIndex = (loopBuffer.readIndex + 1) % loopBuffer.frameCount;
            }
            break;

        case OFFLINE_CHASE:
            if (storedChase.stepCount > 0) {
                ChaseStep* currentStep = &storedChase.steps[chaseCurrentStep];
                ChaseStep* nextStep = &storedChase.steps[(chaseCurrentStep + 1) % storedChase.stepCount];

                unsigned long stepElapsed = now - chaseStepStartTime;
                unsigned long totalStepTime = currentStep->fadeMs + currentStep->holdMs;

                if (chaseFading && stepElapsed < currentStep->fadeMs) {
                    // Fading to current step
                    float progress = stepElapsed / (float)currentStep->fadeMs;
                    uint8_t prevStep = (chaseCurrentStep == 0) ? storedChase.stepCount - 1 : chaseCurrentStep - 1;

                    for (int i = 0; i < LOOP_BUFFER_CHANNELS; i++) {
                        uint8_t from = storedChase.steps[prevStep].channels[i];
                        uint8_t to = currentStep->channels[i];
                        dmxIn[i + 1] = from + (to - from) * progress;
                    }
                } else if (stepElapsed < totalStepTime) {
                    // Holding at current step
                    chaseFading = false;
                    memcpy(dmxIn + 1, currentStep->channels, LOOP_BUFFER_CHANNELS);
                } else {
                    // Move to next step
                    chaseCurrentStep = (chaseCurrentStep + 1) % storedChase.stepCount;
                    chaseStepStartTime = now;
                    chaseFading = true;
                }
            }
            break;
    }
}

void handleChaseCommand(JsonDocument& doc) {
    // Handle chase storage command from Pi
    // Format: {"cmd":"store_chase","steps":[{"channels":[...],"fade_ms":500,"hold_ms":1000},...]}

    JsonArray steps = doc["steps"].as<JsonArray>();
    if (!steps) {
        Serial.println("Chase: No steps array");
        return;
    }

    storedChase.stepCount = 0;
    storedChase.loopCount = doc["loop_count"] | 0;  // 0 = infinite

    for (JsonObject step : steps) {
        if (storedChase.stepCount >= MAX_CHASE_STEPS) break;

        ChaseStep* s = &storedChase.steps[storedChase.stepCount];
        s->fadeMs = step["fade_ms"] | 500;
        s->holdMs = step["hold_ms"] | 1000;

        // Get channel values
        JsonArray channels = step["channels"].as<JsonArray>();
        if (channels) {
            int i = 0;
            for (JsonVariant ch : channels) {
                if (i >= LOOP_BUFFER_CHANNELS) break;
                s->channels[i++] = ch.as<uint8_t>();
            }
        }

        storedChase.stepCount++;
    }

    saveStoredChase();
    Serial.printf("Chase stored: %d steps\n", storedChase.stepCount);
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
    // Only start sACN if paired with a valid universe
    if (!isPaired || sourceUniverse == 0) {
        Serial.println("sACN: Not starting - waiting for pairing");
        return;
    }

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
    if (newUniverse == sourceUniverse && isPaired) return;

    Serial.printf("Universe: %d -> %d\n", sourceUniverse, newUniverse);
    sourceUniverse = newUniverse;

    // Setting a universe means we're now paired
    isPaired = true;

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
            // Mark as paired when config is received (universe must be set)
            if (sourceUniverse > 0) {
                isPaired = true;
            }
            saveConfig();
            Serial.printf("Config updated: Universe=%d, Slice=%d-%d, Mode=%s, Paired=%s\n",
                sourceUniverse, sliceStart, sliceEnd,
                sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
                isPaired ? "YES" : "NO");
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

        // Reset to unpaired state
        isPaired = false;
        sourceUniverse = 0;  // No universe - waiting for config
        sliceStart = 1;
        sliceEnd = 512;
        sliceMode = SLICE_ZERO_OUTSIDE;

        // Generate default name from MAC
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macStr[5];
        sprintf(macStr, "%02X%02X", mac[4], mac[5]);
        nodeName = String("PULSE-") + String(macStr);

        // Clear saved config from NVS
        preferences.begin("aether", false);
        preferences.clear();
        preferences.end();

        // Note: ESPAsyncE131 doesn't have end() method
        // The node will stop processing sACN since sourceUniverse is now 0
        // On reboot, initSacn() won't start since isPaired is false

        Serial.println("Config cleared. Node waiting for pairing.");
        Serial.println("Universe: NONE (waiting for config)");

        // Re-register with Pi to show as unpaired
        sendRegistration();
    }
    // Store chase definition for offline playback
    else if (strcmp(cmd, "store_chase") == 0) {
        handleChaseCommand(doc);
    }
    // Set offline mode
    else if (strcmp(cmd, "set_offline_mode") == 0) {
        const char* mode = doc["mode"];
        if (mode) {
            if (strcmp(mode, "none") == 0) offlineMode = OFFLINE_NONE;
            else if (strcmp(mode, "loop") == 0) offlineMode = OFFLINE_LOOP;
            else if (strcmp(mode, "chase") == 0) offlineMode = OFFLINE_CHASE;
            else if (strcmp(mode, "hold") == 0) offlineMode = OFFLINE_HOLD;

            saveConfig();
            Serial.printf("Offline mode set to: %s\n", mode);
        }
    }
    // Get offline status
    else if (strcmp(cmd, "offline_status") == 0) {
        const char* modeStr = "unknown";
        switch (offlineMode) {
            case OFFLINE_NONE: modeStr = "none"; break;
            case OFFLINE_LOOP: modeStr = "loop"; break;
            case OFFLINE_CHASE: modeStr = "chase"; break;
            case OFFLINE_HOLD: modeStr = "hold"; break;
        }
        Serial.printf("Offline: mode=%s, buffer=%d frames, chase=%d steps, active=%s\n",
                      modeStr, loopBuffer.frameCount, storedChase.stepCount,
                      isOffline ? "YES" : "NO");
    }
    // ═══════════════════════════════════════════════════════════════════
    // DIRECT DMX DATA VIA UDP JSON (Alternative to sACN - more reliable on WiFi)
    // Format: {"cmd":"dmx","ch":[255,128,64,...]} - up to 512 channels
    // This bypasses sACN entirely for networks where sACN has issues
    // ═══════════════════════════════════════════════════════════════════
    else if (strcmp(cmd, "dmx") == 0) {
        JsonArray channels = doc["ch"].as<JsonArray>();
        if (channels) {
            int i = 1;  // DMX channels start at 1
            for (JsonVariant ch : channels) {
                if (i > 512) break;
                dmxIn[i++] = ch.as<uint8_t>();
            }
            dmxDirty = true;
            lastSacnReceived = millis();  // Treat as sACN for timeout purposes

            // If we were offline, come back online
            if (isOffline) {
                stopOfflineMode();
            }

            // Record frame for offline playback
            recordFrame();
        }
    }
    // Full frame DMX data with start channel offset
    // Format: {"cmd":"dmx_frame","start":1,"ch":[255,128,64,...]}
    else if (strcmp(cmd, "dmx_frame") == 0) {
        int startCh = doc["start"] | 1;
        if (startCh < 1) startCh = 1;
        if (startCh > 512) startCh = 512;

        JsonArray channels = doc["ch"].as<JsonArray>();
        if (channels) {
            int i = startCh;
            for (JsonVariant ch : channels) {
                if (i > 512) break;
                dmxIn[i++] = ch.as<uint8_t>();
            }
            dmxDirty = true;
            lastSacnReceived = millis();

            if (isOffline) {
                stopOfflineMode();
            }
            recordFrame();
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
// FADE ENGINE - Non-blocking per-channel fades
// ═══════════════════════════════════════════════════════════════════
void initFadeEngine() {
    for (int i = 0; i < 513; i++) {
        channelFades[i].active = false;
    }
}

void tickFades() {
    // Process active fades at ~50Hz (20ms interval)
    static unsigned long lastTick = 0;
    unsigned long now = millis();
    if (now - lastTick < 20) return;
    lastTick = now;

    for (int ch = 1; ch <= 512; ch++) {
        if (!channelFades[ch].active) continue;

        unsigned long elapsed = now - channelFades[ch].startTime;
        if (elapsed >= channelFades[ch].durationMs) {
            // Fade complete
            dmxIn[ch] = channelFades[ch].targetValue;
            channelFades[ch].active = false;
        } else {
            // Interpolate (linear)
            float progress = (float)elapsed / (float)channelFades[ch].durationMs;
            int start = channelFades[ch].startValue;
            int target = channelFades[ch].targetValue;
            dmxIn[ch] = start + (int)((target - start) * progress);
        }
    }
}

void startChannelFade(int channel, uint8_t targetValue, unsigned long durationMs) {
    if (channel < 1 || channel > 512) return;

    channelFades[channel].startValue = dmxIn[channel];
    channelFades[channel].targetValue = targetValue;
    channelFades[channel].startTime = millis();
    channelFades[channel].durationMs = durationMs;
    channelFades[channel].active = true;
}

void cancelChannelFade(int channel) {
    if (channel >= 1 && channel <= 512) {
        channelFades[channel].active = false;
    }
}

// ═══════════════════════════════════════════════════════════════════
// UDPJSON DMX COMMAND HANDLER (Port 6455)
// ═══════════════════════════════════════════════════════════════════
void handleUdpjsonDmxCommand(const String& jsonStr, IPAddress senderIP) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);

    if (error) {
        static unsigned long lastErrorLog = 0;
        if (millis() - lastErrorLog > 5000) {
            Serial.printf("UDPJSON parse error: %s\n", error.c_str());
            lastErrorLog = millis();
        }
        return;
    }

    const char* msgType = doc["type"];
    if (!msgType) {
        // Check for legacy "cmd" field for backward compatibility
        const char* cmd = doc["cmd"];
        if (cmd) {
            // Handle legacy commands through the config handler
            handleConfigCommand(jsonStr);
            return;
        }
        return;
    }

    int universe = doc["universe"] | 0;

    // Ignore messages for universes we don't serve
    if (universe != 0 && universe != sourceUniverse) {
        return;
    }

    udpjsonPacketsReceived++;
    lastSacnReceived = millis();  // Treat as activity for timeout

    // If we were offline, come back online
    if (isOffline) {
        stopOfflineMode();
    }

    // Handle message types
    if (strcmp(msgType, "set") == 0) {
        // Set channels immediately
        JsonObject channels = doc["channels"].as<JsonObject>();
        if (channels) {
            int updated = 0;
            for (JsonPair kv : channels) {
                int ch = atoi(kv.key().c_str());
                if (ch >= 1 && ch <= 512) {
                    int val = kv.value().as<int>();
                    dmxIn[ch] = constrain(val, 0, 255);
                    cancelChannelFade(ch);  // Cancel any active fade
                    updated++;
                }
            }
            // Rate-limited logging
            static unsigned long lastSetLog = 0;
            if (millis() - lastSetLog > 1000) {
                Serial.printf("UDPJSON: set U%d, %d channels\n", universe, updated);
                lastSetLog = millis();
            }
        }
        recordFrame();  // Record for offline playback

    } else if (strcmp(msgType, "fade") == 0) {
        // Start fade on channels
        unsigned long durationMs = doc["duration_ms"] | 1000;
        JsonObject channels = doc["channels"].as<JsonObject>();
        if (channels) {
            int started = 0;
            for (JsonPair kv : channels) {
                int ch = atoi(kv.key().c_str());
                if (ch >= 1 && ch <= 512) {
                    int targetVal = kv.value().as<int>();
                    startChannelFade(ch, constrain(targetVal, 0, 255), durationMs);
                    started++;
                }
            }
            Serial.printf("UDPJSON: fade U%d, %d channels, %lums\n", universe, started, durationMs);
        }

    } else if (strcmp(msgType, "blackout") == 0) {
        // Blackout all channels
        Serial.printf("UDPJSON: blackout U%d\n", universe);
        for (int ch = 1; ch <= 512; ch++) {
            dmxIn[ch] = 0;
            cancelChannelFade(ch);
        }

    } else if (strcmp(msgType, "ping") == 0) {
        // Respond with pong
        sendPong(senderIP, UDPJSON_DMX_PORT);
    }
}

void sendPong(IPAddress senderIP, int senderPort) {
    char json[512];
    snprintf(json, sizeof(json),
        "{\"type\":\"pong\",\"node_id\":\"%s\","
        "\"universes\":[%d],\"slice_start\":%d,\"slice_end\":%d,"
        "\"slice_mode\":\"%s\",\"version\":\"%s\","
        "\"rssi\":%d,\"uptime_s\":%lu,\"dmx_tx_fps\":%d,"
        "\"rx_udp_packets\":%lu}",
        nodeId.c_str(), sourceUniverse,
        sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
        FIRMWARE_VERSION, WiFi.RSSI(), millis() / 1000,
        DMX_OUTPUT_FPS, udpjsonPacketsReceived);

    udpjsonDmxUdp.beginPacket(senderIP, senderPort);
    udpjsonDmxUdp.print(json);
    udpjsonDmxUdp.endPacket();

    Serial.printf("UDPJSON: pong sent to %s\n", senderIP.toString().c_str());
}

// ═══════════════════════════════════════════════════════════════════
// NETWORK MESSAGES
// ═══════════════════════════════════════════════════════════════════
void sendRegistration() {
    if (WiFi.status() != WL_CONNECTED) return;

    // Build registration JSON with pairing status
    char json[700];
    snprintf(json, sizeof(json),
        "{\"type\":\"register\",\"node_id\":\"%s\",\"hostname\":\"%s\","
        "\"mac\":\"%s\",\"ip\":\"%s\",\"universe\":%d,"
        "\"is_paired\":%s,\"waiting_for_config\":%s,"
        "\"slice_start\":%d,\"slice_end\":%d,\"slice_mode\":\"%s\","
        "\"startChannel\":%d,\"channelCount\":%d,"
        "\"firmware\":\"pulse-sacn\",\"version\":\"%s\","
        "\"transport\":\"sACN\",\"rssi\":%d,\"uptime\":%lu,"
        "\"sender_policy\":\"last-wins\"}",
        nodeId.c_str(), nodeName.c_str(),
        WiFi.macAddress().c_str(), WiFi.localIP().toString().c_str(),
        sourceUniverse,
        isPaired ? "true" : "false",
        isPaired ? "false" : "true",  // waiting_for_config = !isPaired
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
        "\"uptime\":%lu,\"udpjson_pkts\":%lu,\"dmx_frames\":%lu,"
        "\"universe\":%d,\"slice_start\":%d,\"slice_end\":%d,"
        "\"slice_mode\":\"%s\",\"transport\":\"UDPJSON\","
        "\"port\":%d}",
        nodeId.c_str(), WiFi.RSSI(), millis() / 1000,
        udpjsonPacketsReceived, dmxFramesSent, sourceUniverse,
        sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
        UDPJSON_DMX_PORT);

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

    // Header: Device label (permanent PULSE-XXXX identifier) + signal
    display.setCursor(0, 0);
    display.print(deviceLabel);

    if (WiFi.status() == WL_CONNECTED) {
        drawSignalBars(100, 0, WiFi.RSSI());
    } else {
        display.setCursor(100, 0);
        display.print("--");
    }

    // Separator
    display.drawLine(0, 12, 128, 12, SSD1306_WHITE);

    // Check if node is waiting for configuration
    if (!isPaired || sourceUniverse == 0) {
        // WAITING FOR CONFIG mode - show prominent message
        display.setTextSize(1);
        display.setCursor(0, 18);
        display.print("WAITING FOR");
        display.setCursor(0, 28);
        display.print("CONFIGURATION");

        display.setTextSize(1);
        display.setCursor(0, 42);
        display.print("Use AETHER Portal");
        display.setCursor(0, 52);
        display.print("to pair this node");

        display.display();
        return;
    }

    // PAIRED mode - show normal status
    // Universe (large) - supports any universe, not just 1
    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print("U");
    display.print(sourceUniverse);

    // Mode
    display.setTextSize(1);
    display.setCursor(60, 18);
    display.print("UDP");

    // Slice info (shows the channel range this node outputs)
    display.setCursor(60, 28);
    display.printf("%d-%d", sliceStart, sliceEnd);

    // UDPJSON status with activity indicator
    display.setCursor(0, 40);
    unsigned long timeSinceSacn = millis() - lastSacnReceived;
    if (lastSacnReceived == 0) {
        display.print("Waiting for data...");
    } else if (timeSinceSacn < 1000) {
        display.print("UDPJSON: LIVE");
        // Activity indicator
        display.fillCircle(120, 43, 3, SSD1306_WHITE);
    } else if (isOffline) {
        // Show offline mode
        switch (offlineMode) {
            case OFFLINE_LOOP:
                display.print("OFFLINE: LOOP");
                break;
            case OFFLINE_CHASE:
                display.print("OFFLINE: CHASE");
                break;
            case OFFLINE_HOLD:
                display.print("OFFLINE: HOLD");
                break;
            default:
                display.print("OFFLINE: FADE");
        }
    } else if (timeSinceSacn < SACN_TIMEOUT_MS) {
        display.printf("UDP: %lus ago", timeSinceSacn / 1000);
    } else {
        display.print("UDP: TIMEOUT");
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

    // Generate node ID and device label from MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    nodeId = String("pulse-") + String(macStr).substring(8);

    // Device label is permanent identifier shown on OLED (never changes)
    char labelStr[12];
    sprintf(labelStr, "PULSE-%02X%02X", mac[4], mac[5]);
    deviceLabel = String(labelStr);

    // Boot banner
    Serial.println();
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println("  AETHER Pulse - UDPJSON DMX Node");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.printf("  Firmware:  pulse-udpjson v%s\n", FIRMWARE_VERSION);
    Serial.printf("  Node ID:   %s\n", nodeId.c_str());
    Serial.printf("  Transport: UDPJSON on port %d\n", UDPJSON_DMX_PORT);
    Serial.printf("  Output:    %d fps (fixed rate)\n", DMX_OUTPUT_FPS);
    Serial.printf("  Features:  channel-slice, fade-engine, offline-playback\n");
    Serial.println("═══════════════════════════════════════════════════");
    Serial.println();

    // Initialize OLED
    initOLED();

    // Load config from NVS
    loadConfig();

    // Initialize offline playback system
    initOfflinePlayback();

    // Connect to WiFi
    initWiFi();

    // Initialize fade engine
    initFadeEngine();

    if (WiFi.status() == WL_CONNECTED) {
        // Start UDP listeners
        configUdp.begin(CONFIG_PORT);
        discoveryUdp.begin(DISCOVERY_PORT);
        udpjsonDmxUdp.begin(UDPJSON_DMX_PORT);  // Port 6455 for UDPJSON DMX
        Serial.printf("UDP: Config=%d, Discovery=%d, UDPJSON DMX=%d\n", CONFIG_PORT, DISCOVERY_PORT, UDPJSON_DMX_PORT);

        // Initialize sACN (legacy, kept for backward compatibility)
        initSacn();

        // Enable OTA
        initOTA();

        // Announce ourselves
        sendRegistration();
    }

    // Initialize DMX output
    initDmxOutput();

    Serial.println("\n═══════════════════════════════════════════════════");
    if (isPaired && sourceUniverse > 0) {
        Serial.printf("  READY - Slice %d-%d on Universe %d\n", sliceStart, sliceEnd, sourceUniverse);
    } else {
        Serial.println("  WAITING FOR CONFIGURATION");
        Serial.println("  Use AETHER Portal to pair this node");
    }
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

        // If we were offline, come back online
        if (isOffline) {
            stopOfflineMode();
        }

        // Record frame to loop buffer for offline playback
        recordFrame();
    }

    // ─────────────────────────────────────────────────────────────────
    // OFFLINE PLAYBACK - Switch to offline mode after timeout
    // ─────────────────────────────────────────────────────────────────
    // DISABLED:     if (lastSacnReceived > 0 && (now - lastSacnReceived) > SACN_TIMEOUT_MS) {
    // DISABLED:         // Start offline mode if not already
    // DISABLED:         if (!isOffline) {
    // DISABLED:             startOfflineMode();
    // DISABLED:         }
    // DISABLED: 
    // DISABLED:         // Handle offline playback based on mode
    // DISABLED:         if (offlineMode == OFFLINE_NONE) {
    // DISABLED:             // Gradual fade to black (simple linear fade)
    // DISABLED:             static unsigned long lastFadeStep = 0;
    // DISABLED:             if (now - lastFadeStep > 50) {
    // DISABLED:                 bool anyActive = false;
    // DISABLED:                 for (int i = 1; i <= 512; i++) {
    // DISABLED:                     if (dmxIn[i] > 0) {
    // DISABLED:                         dmxIn[i] = (dmxIn[i] > 5) ? dmxIn[i] - 5 : 0;
    // DISABLED:                         anyActive = true;
    // DISABLED:                     }
    // DISABLED:                 }
    // DISABLED:                 lastFadeStep = now;
    // DISABLED:                 if (!anyActive) {
    // DISABLED:                     lastSacnReceived = 0;
    // DISABLED:                     Serial.println("sACN: Fade to black complete");
    // DISABLED:                 }
    // DISABLED:             }
    // DISABLED:         } else {
    // DISABLED:             // Loop or Chase playback
    // DISABLED:             playOfflineFrame();
    // DISABLED:         }
    // DISABLED:     }

    // ─────────────────────────────────────────────────────────────────
    // UDPJSON DMX COMMANDS (Port 6455) - Primary DMX transport
    // ─────────────────────────────────────────────────────────────────
    int udpjsonPacketSize = udpjsonDmxUdp.parsePacket();
    if (udpjsonPacketSize > 0) {
        char buffer[1024];
        int len = udpjsonDmxUdp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';
            handleUdpjsonDmxCommand(String(buffer), udpjsonDmxUdp.remoteIP());
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // UDP CONFIG COMMANDS (Port 8888 - legacy config)
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
    // FADE ENGINE TICK - Process non-blocking fades
    // ─────────────────────────────────────────────────────────────────
    tickFades();

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
