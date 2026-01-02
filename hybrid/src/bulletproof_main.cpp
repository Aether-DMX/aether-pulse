/**
 * AETHER Pulse - Bulletproof UDPJSON DMX Node v4.0.0
 *
 * Production-ready firmware implementing UDPJSON Protocol v2.0
 *
 * DESIGN PRINCIPLES:
 * 1. DMX output NEVER stops - independent of network state
 * 2. No blackout on silence - stale flag only
 * 3. Safe packet handling - drop malformed, never crash
 * 4. MTU-friendly - compact payloads under 1200 bytes
 * 5. Future-ready - protocol versioning, RDM stub, capabilities
 *
 * ARCHITECTURE:
 * - wifi_manager: Connect/reconnect, never blocks DMX
 * - udp_server: Receive packets, size validation
 * - protocol_parser: Validate JSON, parse v1/v2 formats
 * - dmx_engine: Manage channels, fades, slicing
 * - dmx_output_task: Fixed-rate RS-485 output
 * - telemetry: Counters, stats, diagnostics
 *
 * Protocol: UDPJSON v2.0 (see PROTOCOL.md)
 * Port: 6455 (DMX commands)
 * Output: RS-485/DMX @ 40 Hz
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_dmx.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "mbedtls/base64.h"

// ═══════════════════════════════════════════════════════════════════
// VERSION & PROTOCOL
// ═══════════════════════════════════════════════════════════════════
#define FIRMWARE_VERSION "4.0.0"
#define FIRMWARE_FAMILY "pulse-bulletproof"
#define PROTOCOL_VERSION 2

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
const char* WIFI_PASSWORD = "";

const int CONFIG_PORT = 8888;
const int DISCOVERY_PORT = 9999;
const int UDPJSON_DMX_PORT = 6455;
const char* CONTROLLER_IP = "192.168.50.1";

// ═══════════════════════════════════════════════════════════════════
// SAFETY LIMITS (HARD CONSTRAINTS)
// ═══════════════════════════════════════════════════════════════════
#define MAX_UDP_PAYLOAD 1400       // Drop packets larger than this
#define MAX_JSON_BUFFER 2048       // JSON parsing buffer
#define DMX_PACKET_SIZE 513
#define DMX_OUTPUT_FPS 40
#define DMX_OUTPUT_INTERVAL_MS (1000 / DMX_OUTPUT_FPS)
#define STALE_TIMEOUT_MS 3000      // Mark as stale after 3s
#define SEQ_WINDOW_SIZE 256        // Sequence number duplicate detection window
#define STATUS_LOG_INTERVAL_MS 10000
#define HEARTBEAT_INTERVAL_MS 10000
#define WIFI_CHECK_INTERVAL_MS 5000
#define FADE_TICK_INTERVAL_MS 20   // 50 Hz fade updates

// ═══════════════════════════════════════════════════════════════════
// SLICE MODE ENUM
// ═══════════════════════════════════════════════════════════════════
enum SliceMode {
    SLICE_ZERO_OUTSIDE = 0,
    SLICE_PASS_THROUGH = 1
};

// ═══════════════════════════════════════════════════════════════════
// TELEMETRY COUNTERS
// ═══════════════════════════════════════════════════════════════════
struct Telemetry {
    uint32_t rx_total;
    uint32_t rx_bad;
    uint32_t rx_oversize;
    uint32_t rx_parse_fail;
    uint32_t rx_seq_drop;
    uint32_t rx_universe_drop;
    uint32_t rx_v1_legacy;
    uint32_t rx_v2;
    uint32_t tx_dmx_frames;
    uint32_t tx_pong;
    uint32_t tx_ack;
    unsigned long last_packet_time;
    bool is_stale;
};

Telemetry telemetry = {0};

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

ChannelFade channelFades[513];  // 1-indexed

// ═══════════════════════════════════════════════════════════════════
// DMX ENGINE - Channel management
// ═══════════════════════════════════════════════════════════════════
struct DMXEngine {
    uint8_t current[DMX_PACKET_SIZE];   // Current output values
    uint8_t target[DMX_PACKET_SIZE];    // Fade targets
    uint8_t start[DMX_PACKET_SIZE];     // Fade start values
    uint8_t output[DMX_PACKET_SIZE];    // Final assembled output with slicing

    void init() {
        memset(current, 0, sizeof(current));
        memset(target, 0, sizeof(target));
        memset(start, 0, sizeof(start));
        memset(output, 0, sizeof(output));
    }

    void setChannel(int ch, uint8_t value) {
        if (ch >= 1 && ch <= 512) {
            current[ch] = value;
            target[ch] = value;
            channelFades[ch].active = false;
        }
    }

    void startFade(int ch, uint8_t targetValue, unsigned long durationMs) {
        if (ch >= 1 && ch <= 512 && durationMs > 0) {
            channelFades[ch].startValue = current[ch];
            channelFades[ch].targetValue = targetValue;
            channelFades[ch].startTime = millis();
            channelFades[ch].durationMs = durationMs;
            channelFades[ch].active = true;
            target[ch] = targetValue;
            start[ch] = current[ch];
        } else if (ch >= 1 && ch <= 512) {
            setChannel(ch, targetValue);
        }
    }

    void blackout() {
        for (int ch = 1; ch <= 512; ch++) {
            current[ch] = 0;
            target[ch] = 0;
            channelFades[ch].active = false;
        }
    }

    void panic() {
        blackout();
    }
};

DMXEngine dmxEngine;

// ═══════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════
Preferences preferences;
WiFiUDP configUdp;
WiFiUDP discoveryUdp;
WiFiUDP dmxUdp;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledPresent = false;

String nodeId = "";
String nodeName = "";
String deviceLabel = "";

bool isPaired = false;
int sourceUniverse = 0;
int sliceStart = 1;
int sliceEnd = 512;
SliceMode sliceMode = SLICE_ZERO_OUTSIDE;

uint32_t lastSeq = 0;
bool seqInitialized = false;

unsigned long lastDmxSend = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastStatusLog = 0;
unsigned long lastFadeTick = 0;

// ═══════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════
void initWiFi();
void initDmxOutput();
void initOLED();
void initOTA();
void initFadeEngine();
void loadConfig();
void saveConfig();

void handleDmxPacket(char* buffer, int len, IPAddress senderIP);
void handleConfigPacket(char* buffer, int len);

void tickFades();
void assembleOutput();
void outputDmxFrame();
void updateOLED();
void sendHeartbeat();
void sendRegistration();
void sendPong(IPAddress senderIP, int senderPort, uint32_t seq);
void sendAck(IPAddress senderIP, int senderPort, uint32_t seq, bool ok);
void logStatus();

bool validatePacketSize(int size);
bool validateSequence(uint32_t seq);
bool parseV1Packet(JsonDocument& doc);
bool parseV2Packet(JsonDocument& doc);

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION STORAGE
// ═══════════════════════════════════════════════════════════════════
void loadConfig() {
    preferences.begin("aether", true);
    isPaired = preferences.getBool("is_paired", false);
    sourceUniverse = preferences.getInt("universe", 0);
    sliceStart = preferences.getInt("slice_start", 1);
    sliceEnd = preferences.getInt("slice_end", 512);
    sliceMode = (SliceMode)preferences.getInt("slice_mode", SLICE_ZERO_OUTSIDE);
    nodeName = preferences.getString("name", "");
    preferences.end();

    if (!isPaired) sourceUniverse = 0;

    // Validate ranges
    if (sourceUniverse != 0 && (sourceUniverse < 1 || sourceUniverse > 63999)) {
        Serial.printf("Invalid universe %d, resetting\n", sourceUniverse);
        sourceUniverse = 0;
        isPaired = false;
    }
    sliceStart = constrain(sliceStart, 1, 512);
    sliceEnd = constrain(sliceEnd, sliceStart, 512);
    if (sliceMode != SLICE_ZERO_OUTSIDE && sliceMode != SLICE_PASS_THROUGH) {
        sliceMode = SLICE_ZERO_OUTSIDE;
    }

    if (nodeName.length() == 0) {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macStr[5];
        sprintf(macStr, "%02X%02X", mac[4], mac[5]);
        nodeName = String("PULSE-") + String(macStr);
    }

    Serial.printf("Config: Universe=%d, Slice=%d-%d, Mode=%s, Paired=%s\n",
        sourceUniverse, sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero" : "pass",
        isPaired ? "YES" : "NO");
}

void saveConfig() {
    preferences.begin("aether", false);
    preferences.putBool("is_paired", isPaired);
    preferences.putInt("universe", sourceUniverse);
    preferences.putInt("slice_start", sliceStart);
    preferences.putInt("slice_end", sliceEnd);
    preferences.putInt("slice_mode", (int)sliceMode);
    preferences.putString("name", nodeName);
    preferences.end();
    Serial.println("Config saved");
}

// ═══════════════════════════════════════════════════════════════════
// WIFI MANAGER
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
        Serial.printf("\nWiFi connected! IP: %s, RSSI: %d\n",
            WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("\nWiFi failed, will retry...");
    }
}

// ═══════════════════════════════════════════════════════════════════
// PACKET VALIDATION
// ═══════════════════════════════════════════════════════════════════
bool validatePacketSize(int size) {
    if (size > MAX_UDP_PAYLOAD) {
        telemetry.rx_oversize++;
        return false;
    }
    return true;
}

bool validateSequence(uint32_t seq) {
    if (!seqInitialized) {
        lastSeq = seq;
        seqInitialized = true;
        return true;
    }

    // Handle wraparound correctly
    int32_t diff = (int32_t)(seq - lastSeq);

    // If seq is behind lastSeq but within window, it's a duplicate/old packet
    if (diff <= 0 && diff > -SEQ_WINDOW_SIZE) {
        telemetry.rx_seq_drop++;
        return false;
    }

    // Large negative diff likely means wraparound, accept it
    // Large positive diff is a gap (acceptable for UDP)
    lastSeq = seq;
    return true;
}

// ═══════════════════════════════════════════════════════════════════
// PROTOCOL PARSER - V1 Legacy Format
// ═══════════════════════════════════════════════════════════════════
bool parseV1Packet(JsonDocument& doc, IPAddress senderIP) {
    telemetry.rx_v1_legacy++;

    const char* msgType = doc["type"];
    if (!msgType) {
        // Check for legacy "cmd" field
        const char* cmd = doc["cmd"];
        if (cmd) {
            handleConfigPacket(nullptr, 0);  // Route to config handler
            return true;
        }
        return false;
    }

    int universe = doc["universe"] | 0;
    if (universe != 0 && universe != sourceUniverse) {
        telemetry.rx_universe_drop++;
        return false;
    }

    if (strcmp(msgType, "set") == 0) {
        JsonObject channels = doc["channels"].as<JsonObject>();
        if (channels) {
            for (JsonPair kv : channels) {
                int ch = atoi(kv.key().c_str());
                int val = kv.value().as<int>();
                dmxEngine.setChannel(ch, constrain(val, 0, 255));
            }
        }
        return true;
    }
    else if (strcmp(msgType, "fade") == 0) {
        unsigned long durationMs = doc["duration_ms"] | 1000;
        JsonObject channels = doc["channels"].as<JsonObject>();
        if (channels) {
            for (JsonPair kv : channels) {
                int ch = atoi(kv.key().c_str());
                int val = kv.value().as<int>();
                dmxEngine.startFade(ch, constrain(val, 0, 255), durationMs);
            }
        }
        return true;
    }
    else if (strcmp(msgType, "blackout") == 0) {
        dmxEngine.blackout();
        return true;
    }
    else if (strcmp(msgType, "ping") == 0) {
        sendPong(senderIP, UDPJSON_DMX_PORT, 0);
        return true;
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════
// PROTOCOL PARSER - V2 Compact Format
// ═══════════════════════════════════════════════════════════════════
bool parseV2Packet(JsonDocument& doc, IPAddress senderIP) {
    telemetry.rx_v2++;

    const char* msgType = doc["type"];
    if (!msgType) return false;

    uint32_t seq = doc["seq"] | 0;
    int universe = doc["u"] | 0;
    unsigned long fadeMs = doc["fade"] | 0;
    bool wantsAck = doc["ack"] | false;

    // Universe check (0 = broadcast to all)
    if (universe != 0 && universe != sourceUniverse) {
        telemetry.rx_universe_drop++;
        return false;
    }

    // Sequence validation
    if (seq > 0 && !validateSequence(seq)) {
        return false;  // Duplicate/old packet
    }

    bool success = false;

    if (strcmp(msgType, "set") == 0) {
        // Compact format: ch is array of [channel, value] pairs
        JsonArray chArray = doc["ch"].as<JsonArray>();
        if (chArray) {
            for (JsonArray pair : chArray) {
                if (pair.size() >= 2) {
                    int ch = pair[0].as<int>();
                    int val = pair[1].as<int>();
                    if (fadeMs > 0) {
                        dmxEngine.startFade(ch, constrain(val, 0, 255), fadeMs);
                    } else {
                        dmxEngine.setChannel(ch, constrain(val, 0, 255));
                    }
                }
            }
            success = true;
        }
        // Fallback to legacy object format
        else {
            JsonObject channels = doc["channels"].as<JsonObject>();
            if (channels) {
                for (JsonPair kv : channels) {
                    int ch = atoi(kv.key().c_str());
                    int val = kv.value().as<int>();
                    if (fadeMs > 0) {
                        dmxEngine.startFade(ch, constrain(val, 0, 255), fadeMs);
                    } else {
                        dmxEngine.setChannel(ch, constrain(val, 0, 255));
                    }
                }
                success = true;
            }
        }
    }
    else if (strcmp(msgType, "fill") == 0) {
        JsonArray ranges = doc["ranges"].as<JsonArray>();
        if (ranges) {
            for (JsonArray range : ranges) {
                if (range.size() >= 3) {
                    int start = range[0].as<int>();
                    int end = range[1].as<int>();
                    int val = constrain(range[2].as<int>(), 0, 255);
                    start = constrain(start, 1, 512);
                    end = constrain(end, start, 512);
                    for (int ch = start; ch <= end; ch++) {
                        if (fadeMs > 0) {
                            dmxEngine.startFade(ch, val, fadeMs);
                        } else {
                            dmxEngine.setChannel(ch, val);
                        }
                    }
                }
            }
            success = true;
        }
    }
    else if (strcmp(msgType, "frame") == 0) {
        const char* b64 = doc["b64"];
        if (b64) {
            // Decode base64 to raw bytes using mbedtls
            size_t b64_len = strlen(b64);
            uint8_t decoded[512];
            size_t decoded_len = 0;

            int ret = mbedtls_base64_decode(
                decoded, sizeof(decoded), &decoded_len,
                (const unsigned char*)b64, b64_len
            );

            if (ret == 0 && decoded_len == 512) {
                for (int ch = 1; ch <= 512; ch++) {
                    uint8_t val = decoded[ch - 1];
                    if (fadeMs > 0) {
                        dmxEngine.startFade(ch, val, fadeMs);
                    } else {
                        dmxEngine.setChannel(ch, val);
                    }
                }
                success = true;
            } else {
                static unsigned long lastB64Error = 0;
                if (millis() - lastB64Error > 5000) {
                    Serial.printf("Base64 decode error: ret=%d, len=%d\n", ret, decoded_len);
                    lastB64Error = millis();
                }
            }
        }
    }
    else if (strcmp(msgType, "blackout") == 0) {
        if (fadeMs > 0) {
            for (int ch = 1; ch <= 512; ch++) {
                dmxEngine.startFade(ch, 0, fadeMs);
            }
        } else {
            dmxEngine.blackout();
        }
        success = true;
    }
    else if (strcmp(msgType, "panic") == 0) {
        dmxEngine.panic();
        success = true;
    }
    else if (strcmp(msgType, "ping") == 0) {
        sendPong(senderIP, UDPJSON_DMX_PORT, seq);
        success = true;
    }
    else if (strcmp(msgType, "rdm") == 0) {
        // RDM stub - respond with not supported
        char response[128];
        snprintf(response, sizeof(response),
            "{\"v\":%d,\"type\":\"rdm_err\",\"seq\":%lu,\"err\":\"not_supported_yet\"}",
            PROTOCOL_VERSION, seq);
        dmxUdp.beginPacket(senderIP, UDPJSON_DMX_PORT);
        dmxUdp.print(response);
        dmxUdp.endPacket();
        success = true;
    }

    if (wantsAck && seq > 0) {
        sendAck(senderIP, UDPJSON_DMX_PORT, seq, success);
    }

    return success;
}

// ═══════════════════════════════════════════════════════════════════
// PACKET HANDLER
// ═══════════════════════════════════════════════════════════════════
void handleDmxPacket(char* buffer, int len, IPAddress senderIP) {
    telemetry.rx_total++;
    telemetry.last_packet_time = millis();
    telemetry.is_stale = false;

    // Size validation
    if (!validatePacketSize(len)) {
        return;
    }

    // Parse JSON
    StaticJsonDocument<MAX_JSON_BUFFER> doc;
    DeserializationError error = deserializeJson(doc, buffer, len);

    if (error) {
        telemetry.rx_parse_fail++;
        static unsigned long lastErrorLog = 0;
        if (millis() - lastErrorLog > 5000) {
            Serial.printf("JSON parse error: %s\n", error.c_str());
            lastErrorLog = millis();
        }
        return;
    }

    // Check protocol version
    int version = doc["v"] | 1;  // Default to v1 if not specified

    bool success = false;
    if (version >= 2) {
        success = parseV2Packet(doc, senderIP);
    } else {
        success = parseV1Packet(doc, senderIP);
    }

    if (!success) {
        telemetry.rx_bad++;
    }
}

// ═══════════════════════════════════════════════════════════════════
// CONFIG HANDLER (Port 8888)
// ═══════════════════════════════════════════════════════════════════
void handleConfigPacket(char* buffer, int len) {
    if (!buffer || len <= 0) return;

    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, buffer, len);
    if (error) return;

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    Serial.printf("Config cmd: %s\n", cmd);

    if (strcmp(cmd, "config") == 0) {
        bool changed = false;

        if (doc["name"].is<const char*>()) {
            nodeName = doc["name"].as<String>();
            changed = true;
        }
        if (doc["universe"].is<int>()) {
            int u = doc["universe"];
            if (u >= 1 && u <= 63999) {
                sourceUniverse = u;
                isPaired = true;
                changed = true;
            }
        }
        if (doc["slice_start"].is<int>()) {
            sliceStart = constrain(doc["slice_start"].as<int>(), 1, 512);
            changed = true;
        }
        if (doc["slice_end"].is<int>()) {
            sliceEnd = constrain(doc["slice_end"].as<int>(), sliceStart, 512);
            changed = true;
        }
        if (doc["slice_mode"].is<const char*>()) {
            const char* mode = doc["slice_mode"];
            if (strcmp(mode, "zero_outside") == 0) sliceMode = SLICE_ZERO_OUTSIDE;
            else if (strcmp(mode, "pass_through") == 0) sliceMode = SLICE_PASS_THROUGH;
            changed = true;
        }

        if (changed) {
            saveConfig();
            sendRegistration();
        }
    }
    else if (strcmp(cmd, "identify") == 0) {
        for (int i = 0; i < 20; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        logStatus();
        sendRegistration();
    }
    else if (strcmp(cmd, "reboot") == 0) {
        delay(500);
        ESP.restart();
    }
    else if (strcmp(cmd, "unpair") == 0) {
        isPaired = false;
        sourceUniverse = 0;
        sliceStart = 1;
        sliceEnd = 512;
        sliceMode = SLICE_ZERO_OUTSIDE;
        preferences.begin("aether", false);
        preferences.clear();
        preferences.end();
        sendRegistration();
    }
}

// ═══════════════════════════════════════════════════════════════════
// FADE ENGINE TICK
// ═══════════════════════════════════════════════════════════════════
void initFadeEngine() {
    for (int i = 0; i < 513; i++) {
        channelFades[i].active = false;
    }
}

void tickFades() {
    unsigned long now = millis();
    if (now - lastFadeTick < FADE_TICK_INTERVAL_MS) return;
    lastFadeTick = now;

    for (int ch = 1; ch <= 512; ch++) {
        if (!channelFades[ch].active) continue;

        unsigned long elapsed = now - channelFades[ch].startTime;
        if (elapsed >= channelFades[ch].durationMs) {
            dmxEngine.current[ch] = channelFades[ch].targetValue;
            channelFades[ch].active = false;
        } else {
            float progress = (float)elapsed / (float)channelFades[ch].durationMs;
            int start = channelFades[ch].startValue;
            int target = channelFades[ch].targetValue;
            dmxEngine.current[ch] = start + (int)((target - start) * progress);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
// DMX OUTPUT (Fixed-rate, independent of network)
// ═══════════════════════════════════════════════════════════════════
void initDmxOutput() {
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    dmx_driver_install(DMX_PORT, &config, NULL, 0);
    dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_ENABLE_PIN);
    Serial.printf("DMX: Output initialized @ %d fps\n", DMX_OUTPUT_FPS);
}

void assembleOutput() {
    // Apply slice logic: dmxEngine.current -> dmxEngine.output
    if (sliceMode == SLICE_ZERO_OUTSIDE) {
        for (int ch = 1; ch <= 512; ch++) {
            if (ch >= sliceStart && ch <= sliceEnd) {
                dmxEngine.output[ch] = dmxEngine.current[ch];
            } else {
                dmxEngine.output[ch] = 0;
            }
        }
    } else {
        memcpy(dmxEngine.output + 1, dmxEngine.current + 1, 512);
    }
}

void outputDmxFrame() {
    assembleOutput();
    dmx_write(DMX_PORT, dmxEngine.output, DMX_PACKET_SIZE);
    dmx_send(DMX_PORT);
    dmx_wait_sent(DMX_PORT, DMX_TIMEOUT_TICK);
    telemetry.tx_dmx_frames++;
}

// ═══════════════════════════════════════════════════════════════════
// NETWORK RESPONSES
// ═══════════════════════════════════════════════════════════════════
void sendPong(IPAddress senderIP, int senderPort, uint32_t seq) {
    unsigned long now = millis();
    unsigned long staleMs = telemetry.last_packet_time > 0 ?
        now - telemetry.last_packet_time : 0;

    char json[700];
    snprintf(json, sizeof(json),
        "{\"v\":%d,\"type\":\"pong\",\"seq\":%lu,"
        "\"id\":\"%s\",\"u\":%d,\"ip\":\"%s\","
        "\"rssi\":%d,\"uptime\":%lu,\"heap\":%lu,"
        "\"rx\":%lu,\"rx_bad\":%lu,\"rx_oversize\":%lu,"
        "\"rx_seq_drop\":%lu,\"rx_parse_fail\":%lu,\"rx_universe_drop\":%lu,"
        "\"dmx_fps\":%d,\"stale\":%s,\"stale_ms\":%lu,"
        "\"slice\":[%d,%d],"
        "\"caps\":[\"dmx\",\"fade\",\"split\",\"frame\",\"rdm_stub\",\"ota\"]}",
        PROTOCOL_VERSION, seq,
        nodeId.c_str(), sourceUniverse, WiFi.localIP().toString().c_str(),
        WiFi.RSSI(), now / 1000, ESP.getFreeHeap(),
        telemetry.rx_total, telemetry.rx_bad, telemetry.rx_oversize,
        telemetry.rx_seq_drop, telemetry.rx_parse_fail, telemetry.rx_universe_drop,
        DMX_OUTPUT_FPS, telemetry.is_stale ? "true" : "false", staleMs,
        sliceStart, sliceEnd);

    dmxUdp.beginPacket(senderIP, senderPort);
    dmxUdp.print(json);
    dmxUdp.endPacket();
    telemetry.tx_pong++;

    Serial.printf("PONG sent to %s\n", senderIP.toString().c_str());
}

void sendAck(IPAddress senderIP, int senderPort, uint32_t seq, bool ok) {
    char json[128];
    snprintf(json, sizeof(json),
        "{\"v\":%d,\"type\":\"ack\",\"seq\":%lu,\"ok\":%s}",
        PROTOCOL_VERSION, seq, ok ? "true" : "false");

    dmxUdp.beginPacket(senderIP, senderPort);
    dmxUdp.print(json);
    dmxUdp.endPacket();
    telemetry.tx_ack++;
}

void sendRegistration() {
    if (WiFi.status() != WL_CONNECTED) return;

    char json[700];
    snprintf(json, sizeof(json),
        "{\"type\":\"register\",\"node_id\":\"%s\",\"hostname\":\"%s\","
        "\"mac\":\"%s\",\"ip\":\"%s\",\"u\":%d,"
        "\"is_paired\":%s,\"waiting_for_config\":%s,"
        "\"slice\":[%d,%d],\"slice_mode\":\"%s\","
        "\"firmware\":\"%s\",\"version\":\"%s\","
        "\"protocol_version\":%d,"
        "\"transport\":\"UDPJSON\",\"rssi\":%d,\"uptime\":%lu,"
        "\"caps\":[\"dmx\",\"fade\",\"split\",\"frame\",\"rdm_stub\",\"ota\"]}",
        nodeId.c_str(), nodeName.c_str(),
        WiFi.macAddress().c_str(), WiFi.localIP().toString().c_str(),
        sourceUniverse,
        isPaired ? "true" : "false",
        isPaired ? "false" : "true",
        sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero_outside" : "pass_through",
        FIRMWARE_FAMILY, FIRMWARE_VERSION,
        PROTOCOL_VERSION,
        WiFi.RSSI(), millis() / 1000);

    discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
    discoveryUdp.print(json);
    discoveryUdp.endPacket();
}

void sendHeartbeat() {
    if (WiFi.status() != WL_CONNECTED) return;

    char json[400];
    snprintf(json, sizeof(json),
        "{\"type\":\"heartbeat\",\"node_id\":\"%s\",\"u\":%d,"
        "\"rssi\":%d,\"uptime\":%lu,"
        "\"rx\":%lu,\"tx_dmx\":%lu,"
        "\"slice\":[%d,%d],\"stale\":%s,"
        "\"transport\":\"UDPJSON\",\"protocol_version\":%d}",
        nodeId.c_str(), sourceUniverse,
        WiFi.RSSI(), millis() / 1000,
        telemetry.rx_total, telemetry.tx_dmx_frames,
        sliceStart, sliceEnd,
        telemetry.is_stale ? "true" : "false",
        PROTOCOL_VERSION);

    discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
    discoveryUdp.print(json);
    discoveryUdp.endPacket();
}

// ═══════════════════════════════════════════════════════════════════
// STATUS LOGGING
// ═══════════════════════════════════════════════════════════════════
void logStatus() {
    unsigned long now = millis();
    unsigned long staleMs = telemetry.last_packet_time > 0 ?
        now - telemetry.last_packet_time : 0;

    Serial.println("-------------------------------------------");
    Serial.printf("STATUS @ %lus | %s | Universe %d\n",
        now / 1000, nodeId.c_str(), sourceUniverse);
    Serial.println("-------------------------------------------");
    Serial.printf("  Slice:     %d-%d (%s)\n", sliceStart, sliceEnd,
        sliceMode == SLICE_ZERO_OUTSIDE ? "zero" : "pass");
    Serial.printf("  Protocol:  v%d\n", PROTOCOL_VERSION);
    Serial.printf("  Stale:     %s (%lums)\n",
        telemetry.is_stale ? "YES" : "NO", staleMs);
    Serial.printf("  RX Total:  %lu (v1:%lu, v2:%lu)\n",
        telemetry.rx_total, telemetry.rx_v1_legacy, telemetry.rx_v2);
    Serial.printf("  RX Errors: bad=%lu oversize=%lu parse=%lu seq=%lu univ=%lu\n",
        telemetry.rx_bad, telemetry.rx_oversize, telemetry.rx_parse_fail,
        telemetry.rx_seq_drop, telemetry.rx_universe_drop);
    Serial.printf("  TX DMX:    %lu frames (%.1f fps)\n",
        telemetry.tx_dmx_frames,
        telemetry.tx_dmx_frames * 1000.0 / now);
    Serial.printf("  WiFi:      %s, RSSI=%d\n",
        WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED", WiFi.RSSI());
    Serial.printf("  Heap:      %lu bytes free\n", ESP.getFreeHeap());
    Serial.printf("  Preview:   ch1-4=[%d,%d,%d,%d]\n",
        dmxEngine.output[1], dmxEngine.output[2],
        dmxEngine.output[3], dmxEngine.output[4]);
    Serial.println("-------------------------------------------");
}

// ═══════════════════════════════════════════════════════════════════
// OLED DISPLAY
// ═══════════════════════════════════════════════════════════════════
void initOLED() {
    Wire.begin(OLED_SDA, OLED_SCL);

    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledPresent = true;
    } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        oledPresent = true;
    } else {
        Serial.println("OLED: Not detected");
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(20, 8);
    display.print("AETHER");
    display.setTextSize(1);
    display.setCursor(25, 30);
    display.print("BULLETPROOF");
    display.setCursor(30, 45);
    display.printf("v%s", FIRMWARE_VERSION);
    display.display();
    delay(1000);
}

void drawSignalBars(int x, int y, int rssi) {
    int bars = rssi > -50 ? 4 : rssi > -60 ? 3 : rssi > -70 ? 2 : rssi > -80 ? 1 : 0;
    for (int i = 0; i < 4; i++) {
        int h = 3 + (i * 2);
        int barY = y + (8 - h);
        if (i < bars) display.fillRect(x + (i * 4), barY, 3, h, SSD1306_WHITE);
        else display.drawRect(x + (i * 4), barY, 3, h, SSD1306_WHITE);
    }
}

void updateOLED() {
    if (!oledPresent) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.print(deviceLabel);

    if (WiFi.status() == WL_CONNECTED) {
        drawSignalBars(100, 0, WiFi.RSSI());
    } else {
        display.setCursor(100, 0);
        display.print("--");
    }

    display.drawLine(0, 12, 128, 12, SSD1306_WHITE);

    if (!isPaired || sourceUniverse == 0) {
        display.setTextSize(1);
        display.setCursor(0, 18);
        display.print("WAITING FOR");
        display.setCursor(0, 28);
        display.print("CONFIGURATION");
        display.setCursor(0, 45);
        display.print("Pair via Portal");
    } else {
        display.setTextSize(2);
        display.setCursor(0, 18);
        display.print("U");
        display.print(sourceUniverse);

        display.setTextSize(1);
        display.setCursor(60, 18);
        display.print("v2");
        display.setCursor(60, 28);
        display.printf("%d-%d", sliceStart, sliceEnd);

        display.setCursor(0, 40);
        if (telemetry.is_stale) {
            display.print("STALE");
        } else if (telemetry.last_packet_time == 0) {
            display.print("Waiting...");
        } else {
            display.print("LIVE");
            display.fillCircle(35, 43, 3, SSD1306_WHITE);
        }

        display.setCursor(0, 52);
        display.printf("RX:%lu TX:%lu",
            telemetry.rx_total % 100000,
            telemetry.tx_dmx_frames % 100000);
    }

    display.display();
}

// ═══════════════════════════════════════════════════════════════════
// OTA UPDATES
// ═══════════════════════════════════════════════════════════════════
void initOTA() {
    ArduinoOTA.setHostname(nodeName.c_str());

    ArduinoOTA.onStart([]() {
        Serial.println("OTA: Starting...");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA: Complete!");
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

    // Initialize engine
    dmxEngine.init();
    initFadeEngine();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Generate node IDs
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    nodeId = String("pulse-") + String(macStr).substring(8);
    deviceLabel = String("PULSE-") + String(macStr + 8, 4);

    // Banner
    Serial.println();
    Serial.println("===================================================");
    Serial.println("  AETHER Pulse - BULLETPROOF UDPJSON Node");
    Serial.println("===================================================");
    Serial.printf("  Firmware:  %s v%s\n", FIRMWARE_FAMILY, FIRMWARE_VERSION);
    Serial.printf("  Protocol:  v%d\n", PROTOCOL_VERSION);
    Serial.printf("  Node ID:   %s\n", nodeId.c_str());
    Serial.printf("  Port:      %d\n", UDPJSON_DMX_PORT);
    Serial.printf("  Output:    %d fps\n", DMX_OUTPUT_FPS);
    Serial.println("===================================================");
    Serial.println();

    initOLED();
    loadConfig();
    initWiFi();

    if (WiFi.status() == WL_CONNECTED) {
        configUdp.begin(CONFIG_PORT);
        discoveryUdp.begin(DISCOVERY_PORT);
        dmxUdp.begin(UDPJSON_DMX_PORT);
        Serial.printf("UDP: Config=%d, Discovery=%d, DMX=%d\n",
            CONFIG_PORT, DISCOVERY_PORT, UDPJSON_DMX_PORT);

        initOTA();
        sendRegistration();
    }

    initDmxOutput();

    Serial.println();
    if (isPaired && sourceUniverse > 0) {
        Serial.printf("READY: Universe %d, Slice %d-%d\n",
            sourceUniverse, sliceStart, sliceEnd);
    } else {
        Serial.println("WAITING FOR CONFIGURATION");
    }
    Serial.println();

    digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
    unsigned long now = millis();

    // ─────────────────────────────────────────────────────────────────
    // UDP DMX COMMANDS (Port 6455)
    // ─────────────────────────────────────────────────────────────────
    int packetSize = dmxUdp.parsePacket();
    if (packetSize > 0) {
        static char buffer[MAX_JSON_BUFFER];
        int len = dmxUdp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';
            handleDmxPacket(buffer, len, dmxUdp.remoteIP());
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // UDP CONFIG COMMANDS (Port 8888)
    // ─────────────────────────────────────────────────────────────────
    packetSize = configUdp.parsePacket();
    if (packetSize > 0) {
        static char configBuffer[1024];
        int len = configUdp.read(configBuffer, sizeof(configBuffer) - 1);
        if (len > 0) {
            configBuffer[len] = '\0';
            handleConfigPacket(configBuffer, len);
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // STALE CHECK (status only, no blackout)
    // ─────────────────────────────────────────────────────────────────
    if (telemetry.last_packet_time > 0 &&
        now - telemetry.last_packet_time > STALE_TIMEOUT_MS) {
        if (!telemetry.is_stale) {
            telemetry.is_stale = true;
            Serial.println("STALE: No packets received recently");
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // FADE ENGINE TICK
    // ─────────────────────────────────────────────────────────────────
    tickFades();

    // ─────────────────────────────────────────────────────────────────
    // DMX OUTPUT @ FIXED RATE (NEVER STOPS)
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
    // HEARTBEAT
    // ─────────────────────────────────────────────────────────────────
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        sendHeartbeat();
        lastHeartbeat = now;
    }

    // ─────────────────────────────────────────────────────────────────
    // WIFI RECONNECT
    // ─────────────────────────────────────────────────────────────────
    if (now - lastWifiCheck >= WIFI_CHECK_INTERVAL_MS) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost, reconnecting...");
            WiFi.reconnect();
        }
        lastWifiCheck = now;
    }

    // ─────────────────────────────────────────────────────────────────
    // OLED UPDATE
    // ─────────────────────────────────────────────────────────────────
    if (now - lastDisplayUpdate >= 1000) {
        updateOLED();
        lastDisplayUpdate = now;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    // ─────────────────────────────────────────────────────────────────
    // STATUS LOG
    // ─────────────────────────────────────────────────────────────────
    if (now - lastStatusLog >= STATUS_LOG_INTERVAL_MS) {
        logStatus();
        lastStatusLog = now;
    }

    yield();
}
