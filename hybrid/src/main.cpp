/**
 * AETHER Pulse DMX Node
 * Supports multiple transport modes via build flags:
 *   - ENABLE_SACN: sACN/E1.31 + UDP JSON (hybrid mode)
 *   - ENABLE_ESPNOW: ESP-NOW broadcast (low-latency mode)
 *
 * Features:
 * - Auto-detect: checks for Pi UART connection on boot
 * - Wired mode: receives JSON commands from Pi via UART
 * - Wireless mode: receives sACN/E1.31 + UDP commands (or ESP-NOW)
 * - Local scene/chase storage and playback
 * - Hold-last-look when connection lost
 * - Fade engine with smooth transitions
 * - OTA updates (wireless mode only)
 * - Firmware identity + OTA channel gating
 */

#include <Arduino.h>
#include <esp_dmx.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "firmware_identity.h"

// Conditional includes based on transport mode
#ifdef ENABLE_SACN
#include <ESPAsyncE131.h>
#endif

// ESP-NOW node mode (receive ESP-NOW broadcasts)
#ifdef ENABLE_ESPNOW
#include "transport.h"  // For Transport namespace
#endif

// ESP-NOW Gateway mode (rebroadcast sACN/UART to ESP-NOW nodes)
#ifdef ENABLE_ESPNOW_GATEWAY
#include "transport.h"  // For EspNowGateway namespace
#endif
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════

#ifdef ENABLE_ESPNOW_GATEWAY
// Gateway mode: Pi UART on GPIO16/17, DMX on GPIO18/19
// This allows both Pi comms AND local DMX output simultaneously
#define DMX_TX_PIN 19
#define DMX_RX_PIN 18
#define DMX_ENABLE_PIN 4
#define DMX_PORT 2        // Use UART2 for DMX

#define PI_RX_PIN 16      // Pi TX -> ESP RX (UART1)
#define PI_TX_PIN 17      // ESP TX -> Pi RX (for future RDM)
#else
// Node mode: DMX on GPIO16/17 (no Pi connection)
#define DMX_TX_PIN 17
#define DMX_RX_PIN 16
#define DMX_ENABLE_PIN 4
#define DMX_PORT 1

#define PI_RX_PIN 16      // Same as DMX_RX - reused in wired mode
#endif

#define LED_PIN 2
#define OLED_SDA 21
#define OLED_SCL 22

// ═══════════════════════════════════════════════════════════════
// NETWORK CONFIGURATION
// ═══════════════════════════════════════════════════════════════
const char* WIFI_SSID = "AetherDMX";
const char* WIFI_PASSWORD = "";
const int DISCOVERY_PORT = 9999;
const int CONFIG_PORT = 8888;
const char* CONTROLLER_IP = "192.168.50.1";

// ═══════════════════════════════════════════════════════════════
// DMX CONSTANTS
// ═══════════════════════════════════════════════════════════════
#define DMX_BREAK_US 176
#define DMX_MAB_US 12
#define DMX_PACKET_SIZE 513
#define DMX_REFRESH_HZ 40

// ═══════════════════════════════════════════════════════════════
// MULTI-UNIVERSE GATEWAY SUPPORT
// ═══════════════════════════════════════════════════════════════
#ifdef ENABLE_ESPNOW_GATEWAY
#define MAX_UNIVERSES 8          // Support universes 1-8
#define UNIVERSE_KEEPALIVE_MS 500 // Broadcast unchanged universes every 500ms (2Hz)
#define UNIVERSE_TIMEOUT_MS 30000 // Consider universe inactive after 30s no updates

struct UniverseBuffer {
  uint8_t data[512];              // DMX channel data
  uint32_t lastUpdateTime;        // When Pi last sent data for this universe
  uint32_t lastBroadcastTime;     // When we last broadcast this universe
  uint32_t seq;                   // Sequence number for this universe
  uint32_t framesReceived;        // Frames received from Pi
  uint32_t framesBroadcast;       // Frames broadcast via ESP-NOW
  bool dirty;                     // True if data changed since last broadcast
  bool active;                    // True if universe has been used
};

UniverseBuffer universeBuffers[MAX_UNIVERSES];  // Index 0 = Universe 1, etc.

// Helper to get universe buffer (returns nullptr if invalid)
UniverseBuffer* getUniverseBuffer(int universe) {
  if (universe < 1 || universe > MAX_UNIVERSES) return nullptr;
  return &universeBuffers[universe - 1];
}

// Initialize all universe buffers
void initUniverseBuffers() {
  for (int i = 0; i < MAX_UNIVERSES; i++) {
    memset(universeBuffers[i].data, 0, 512);
    universeBuffers[i].lastUpdateTime = 0;
    universeBuffers[i].lastBroadcastTime = 0;
    universeBuffers[i].seq = 0;
    universeBuffers[i].framesReceived = 0;
    universeBuffers[i].framesBroadcast = 0;
    universeBuffers[i].dirty = false;
    universeBuffers[i].active = false;
  }
  Serial.printf("Gateway: Initialized %d universe buffers\n", MAX_UNIVERSES);
}
#endif

// ═══════════════════════════════════════════════════════════════
// OLED
// ═══════════════════════════════════════════════════════════════
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledPresent = false;

// ═══════════════════════════════════════════════════════════════
// OPERATION MODE
// ═══════════════════════════════════════════════════════════════
enum OperationMode {
  MODE_UNKNOWN,
  MODE_WIRED,
  MODE_WIRELESS
};

OperationMode operationMode = MODE_UNKNOWN;

// ═══════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════
Preferences preferences;
WiFiUDP discoveryUdp;
WiFiUDP configUdp;
#ifdef ENABLE_SACN
ESPAsyncE131* e131 = nullptr;
#endif
HardwareSerial DmxSerial(2);   // UART2 for DMX output
HardwareSerial PiSerial(1);    // UART1 for Pi commands (wired mode)

uint8_t dmxData[DMX_PACKET_SIZE] = {0};
String nodeId = "";
String nodeName = "";

// Configuration (stored in flash)
int currentUniverse = 1;
int channelStart = 1;
int channelEnd = 512;
bool isPaired = false;

// Fade engine
struct Fade {
  uint8_t start;
  uint8_t target;
  unsigned long startTime;
  unsigned long duration;
  bool active;
} fades[DMX_PACKET_SIZE];
bool fadeActive = false;  // True if any channel is fading

// Scene storage (up to 10 scenes)
#define MAX_SCENES 10
#define MAX_SCENE_CHANNELS 64
struct StoredScene {
  char id[32];
  char name[32];
  uint16_t channels[MAX_SCENE_CHANNELS];
  uint8_t values[MAX_SCENE_CHANNELS];
  int channelCount;
  int fadeMs;
  bool valid;
} scenes[MAX_SCENES];

// Chase storage (up to 5 chases)
#define MAX_CHASES 5
#define MAX_CHASE_STEPS 16
#define MAX_STEP_CHANNELS 32
struct ChaseStep {
  uint16_t channels[MAX_STEP_CHANNELS];
  uint8_t values[MAX_STEP_CHANNELS];
  int channelCount;
};
struct StoredChase {
  char id[32];
  char name[32];
  ChaseStep steps[MAX_CHASE_STEPS];
  int stepCount;
  int bpm;
  bool loop;
  bool valid;
} chases[MAX_CHASES];

// Playback state
bool isPlayingChase = false;
int currentChaseIndex = -1;
int currentChaseStep = 0;
unsigned long lastChaseStepTime = 0;
unsigned long chaseStepInterval = 500;

// Stats
unsigned long dmxFramesSent = 0;
unsigned long sacnPacketsReceived = 0;
unsigned long commandsReceived = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastDmxSend = 0;
unsigned long lastSacnReceived = 0;
unsigned long lastPiData = 0;

// Forward declarations
void sendRegistration();
#ifdef ENABLE_SACN
void initSacn();
#endif
void saveConfig();
void clearConfig();
void sendDMXFrame();
void processFades();
void processChase();
void startFade(int channel, uint8_t targetValue, unsigned long durationMs);
void handleJsonCommand(const String& jsonStr);
void initOLED();
void updateOLED();
void initOTA();

// ═══════════════════════════════════════════════════════════════
// MODE DETECTION
// ═══════════════════════════════════════════════════════════════
OperationMode detectMode() {
  // Check if Pi is sending data on UART
  // Send a probe and wait for response, or check for existing data

  Serial.println("Detecting operation mode...");

#ifdef ENABLE_ESPNOW_GATEWAY
  // Gateway firmware ALWAYS runs in wired mode (receives from Pi UART)
  // It doesn't support sACN/wireless input - only ESP-NOW broadcast output
  Serial.println("  -> Gateway firmware: WIRED mode (Pi UART input)");
  return MODE_WIRED;

#elif defined(ENABLE_ESPNOW)
  // ESP-NOW node firmware
  // If not paired: connect to WiFi for discovery/pairing (like hybrid nodes)
  // If paired: use ESP-NOW broadcast mode (no WiFi AP connection)

  if (!isPaired) {
    Serial.println("  -> ESP-NOW node: NOT PAIRED - entering WiFi pairing mode");
    // Connect to WiFi for pairing via the normal discovery flow
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, strlen(WIFI_PASSWORD) > 0 ? WIFI_PASSWORD : nullptr);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(500);
      Serial.print(".");
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Blink during connection
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n  -> WiFi connected for pairing: %s\n", WiFi.localIP().toString().c_str());
      return MODE_WIRELESS;  // Use WiFi mode for pairing
    } else {
      Serial.println("\n  -> WiFi failed, waiting for pairing...");
      // Stay in wireless mode but keep trying
      return MODE_WIRELESS;
    }
  } else {
    Serial.println("  -> ESP-NOW node: PAIRED - using ESP-NOW receiver mode");
    return MODE_WIRELESS;  // Will use ESP-NOW instead of WiFi
  }

#else
  // Hybrid (sACN) node firmware: detect based on Pi connection or WiFi availability

  // Initialize Pi serial temporarily to check for data
  PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, -1);
  delay(500);

  // Check if we receive any data from Pi within 2 seconds
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (PiSerial.available()) {
      Serial.println("  -> Pi UART data detected!");
      return MODE_WIRED;
    }
    delay(10);
  }

  // No Pi data, try WiFi
  Serial.println("  -> No Pi UART, trying WiFi...");
  PiSerial.end();

  // Try to connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, strlen(WIFI_PASSWORD) > 0 ? WIFI_PASSWORD : nullptr);

  start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n  -> WiFi connected, using WIRELESS mode");
    return MODE_WIRELESS;
  }

  // Fallback: check stored preference
  preferences.begin("aether", true);
  int storedMode = preferences.getInt("mode", MODE_WIRELESS);
  preferences.end();

  Serial.printf("  -> Using stored mode: %s\n", storedMode == MODE_WIRED ? "WIRED" : "WIRELESS");
  return (OperationMode)storedMode;
#endif
}

// ═══════════════════════════════════════════════════════════════
// CONFIGURATION STORAGE
// ═══════════════════════════════════════════════════════════════
void loadConfig() {
  preferences.begin("aether", true);
  currentUniverse = preferences.getInt("universe", 1);
  channelStart = preferences.getInt("ch_start", 1);
  channelEnd = preferences.getInt("ch_end", 512);
  isPaired = preferences.getBool("paired", false);
  nodeName = preferences.getString("name", "");
  preferences.end();

  if (nodeName.length() == 0) {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[5];
    sprintf(macStr, "%02X%02X", mac[4], mac[5]);
    nodeName = String("AETHER-") + String(macStr);
  }

  Serial.printf("Config: Universe=%d, Ch=%d-%d, Paired=%s\n",
    currentUniverse, channelStart, channelEnd, isPaired ? "Yes" : "No");
}

void saveConfig() {
  preferences.begin("aether", false);
  preferences.putInt("universe", currentUniverse);
  preferences.putInt("ch_start", channelStart);
  preferences.putInt("ch_end", channelEnd);
  preferences.putBool("paired", isPaired);
  preferences.putString("name", nodeName);
  preferences.putInt("mode", operationMode);
  preferences.end();
}

void clearConfig() {
  preferences.begin("aether", false);
  preferences.clear();
  preferences.end();

  // Reset to defaults
  currentUniverse = 1;
  channelStart = 1;
  channelEnd = 512;
  isPaired = false;

  // Reset name to default based on MAC
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[5];
  sprintf(macStr, "%02X%02X", mac[4], mac[5]);
  nodeName = String("AETHER-") + String(macStr);

  Serial.println("✓ Config cleared - reset to defaults");
}

// ═══════════════════════════════════════════════════════════════
// sACN MANAGEMENT (Wireless mode - only when ENABLE_SACN)
// ═══════════════════════════════════════════════════════════════
#ifdef ENABLE_SACN
void initSacn() {
  if (e131 != nullptr) {
    delete e131;
    e131 = nullptr;
  }

  e131 = new ESPAsyncE131(1);

  if (e131->begin(E131_MULTICAST, currentUniverse, 1)) {
    Serial.printf("sACN listening on universe %d\n", currentUniverse);
  } else {
    Serial.println("sACN init failed!");
  }
}
#endif

void changeUniverse(int newUniverse) {
  if (newUniverse < 1 || newUniverse > 63999) return;
  if (newUniverse == currentUniverse) return;

  Serial.printf("Universe: %d -> %d\n", currentUniverse, newUniverse);
  currentUniverse = newUniverse;

#ifdef ENABLE_SACN
  if (operationMode == MODE_WIRELESS) {
    initSacn();
  }
#endif

  saveConfig();
  sendRegistration();
}

// ═══════════════════════════════════════════════════════════════
// NETWORK (Wireless mode)
// ═══════════════════════════════════════════════════════════════
void connectWiFi() {
  Serial.printf("Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  if (strlen(WIFI_PASSWORD) == 0) {
    WiFi.begin(WIFI_SSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
    digitalWrite(LED_PIN, attempts % 2);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    discoveryUdp.begin(DISCOVERY_PORT);
    configUdp.begin(CONFIG_PORT);
  } else {
    Serial.println("\nWiFi FAILED!");
  }
}

void sendRegistration() {
  if (operationMode != MODE_WIRELESS || WiFi.status() != WL_CONNECTED) return;

  char json[512];
  snprintf(json, sizeof(json),
    "{\"type\":\"register\",\"node_id\":\"%s\",\"hostname\":\"%s\","
    "\"mac\":\"%s\",\"ip\":\"%s\",\"universe\":%d,"
    "\"startChannel\":%d,\"channelCount\":%d,"
    "\"version\":\"hybrid-1.3\",\"rssi\":%d,\"uptime\":%lu,\"paired\":%s}",
    nodeId.c_str(), nodeName.c_str(),
    WiFi.macAddress().c_str(), WiFi.localIP().toString().c_str(),
    currentUniverse, channelStart, channelEnd - channelStart + 1,
    WiFi.RSSI(), millis() / 1000, isPaired ? "true" : "false");

  discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
  discoveryUdp.print(json);
  discoveryUdp.endPacket();
}

void sendHeartbeat() {
  if (operationMode != MODE_WIRELESS || WiFi.status() != WL_CONNECTED) return;

  char json[256];
  snprintf(json, sizeof(json),
    "{\"type\":\"heartbeat\",\"node_id\":\"%s\",\"rssi\":%d,"
    "\"uptime\":%lu,\"sacn_pkts\":%lu,\"dmx_frames\":%lu,"
    "\"universe\":%d,\"mode\":\"hybrid\"}",
    nodeId.c_str(), WiFi.RSSI(), millis() / 1000,
    sacnPacketsReceived, dmxFramesSent, currentUniverse);

  discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
  discoveryUdp.print(json);
  discoveryUdp.endPacket();
}

// ═══════════════════════════════════════════════════════════════
// DMX OUTPUT
// ═══════════════════════════════════════════════════════════════
void initDmxOutput() {
  if (operationMode == MODE_WIRED) {
    // Software DMX with manual BREAK timing
    DmxSerial.begin(250000, SERIAL_8N2, -1, DMX_TX_PIN);
    pinMode(DMX_ENABLE_PIN, OUTPUT);
    digitalWrite(DMX_ENABLE_PIN, HIGH);
    Serial.println("DMX: Software mode (wired)");
  } else {
    // Use esp_dmx library for wireless
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    dmx_driver_install(DMX_PORT, &config, NULL, 0);
    dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_ENABLE_PIN);
    Serial.println("DMX: esp_dmx library (wireless)");
  }
}

void sendDMXFrame() {
  if (operationMode == MODE_WIRED) {
    // Manual BREAK/MAB timing for wired mode
    DmxSerial.end();
    pinMode(DMX_TX_PIN, OUTPUT);
    digitalWrite(DMX_TX_PIN, LOW);
    delayMicroseconds(DMX_BREAK_US);
    digitalWrite(DMX_TX_PIN, HIGH);
    delayMicroseconds(DMX_MAB_US);
    DmxSerial.begin(250000, SERIAL_8N2, -1, DMX_TX_PIN);
    DmxSerial.write(dmxData, DMX_PACKET_SIZE);
    DmxSerial.flush();
  } else {
    // esp_dmx library for wireless
    dmx_write(DMX_PORT, dmxData, DMX_PACKET_SIZE);
    dmx_send(DMX_PORT);
    dmx_wait_sent(DMX_PORT, DMX_TIMEOUT_TICK);
  }

  dmxFramesSent++;
}

// ═══════════════════════════════════════════════════════════════
// FADE ENGINE
// ═══════════════════════════════════════════════════════════════
void startFade(int channel, uint8_t targetValue, unsigned long durationMs) {
  if (channel < 1 || channel > 512) return;

  if (durationMs == 0) {
    dmxData[channel] = targetValue;
    fades[channel].active = false;
  } else {
    fades[channel].start = dmxData[channel];
    fades[channel].target = targetValue;
    fades[channel].startTime = millis();
    fades[channel].duration = durationMs;
    fades[channel].active = true;
  }
}

void processFades() {
  unsigned long now = millis();
  bool anyActive = false;

  for (int i = 1; i <= 512; i++) {
    if (!fades[i].active) continue;

    anyActive = true;
    unsigned long elapsed = now - fades[i].startTime;

    if (elapsed >= fades[i].duration) {
      dmxData[i] = fades[i].target;
      fades[i].active = false;
    } else {
      float progress = (float)elapsed / fades[i].duration;
      float eased = 0.5f - 0.5f * cosf(progress * PI);
      int range = (int)fades[i].target - (int)fades[i].start;
      dmxData[i] = (uint8_t)(fades[i].start + (int)(range * eased));
    }
  }
  fadeActive = anyActive;
}

// ═══════════════════════════════════════════════════════════════
// SCENE/CHASE STORAGE
// ═══════════════════════════════════════════════════════════════
int findSceneSlot(const char* id) {
  // Find existing or empty slot
  int emptySlot = -1;
  for (int i = 0; i < MAX_SCENES; i++) {
    if (scenes[i].valid && strcmp(scenes[i].id, id) == 0) return i;
    if (!scenes[i].valid && emptySlot < 0) emptySlot = i;
  }
  return emptySlot;
}

int findChaseSlot(const char* id) {
  int emptySlot = -1;
  for (int i = 0; i < MAX_CHASES; i++) {
    if (chases[i].valid && strcmp(chases[i].id, id) == 0) return i;
    if (!chases[i].valid && emptySlot < 0) emptySlot = i;
  }
  return emptySlot;
}

void storeScene(JsonDocument& doc) {
  const char* id = doc["id"];
  if (!id) return;

  int slot = findSceneSlot(id);
  if (slot < 0) {
    Serial.println("No scene slots available!");
    return;
  }

  strncpy(scenes[slot].id, id, 31);
  strncpy(scenes[slot].name, doc["name"] | "Scene", 31);
  scenes[slot].fadeMs = doc["fade_ms"] | 500;
  scenes[slot].channelCount = 0;

  JsonObject channels = doc["channels"];
  if (channels) {
    for (JsonPair kv : channels) {
      if (scenes[slot].channelCount >= MAX_SCENE_CHANNELS) break;
      int idx = scenes[slot].channelCount;
      scenes[slot].channels[idx] = atoi(kv.key().c_str());
      scenes[slot].values[idx] = kv.value().as<uint8_t>();
      scenes[slot].channelCount++;
    }
  }

  scenes[slot].valid = true;
  Serial.printf("Stored scene '%s' (%d channels)\n", scenes[slot].name, scenes[slot].channelCount);
}

void storeChase(JsonDocument& doc) {
  const char* id = doc["id"];
  if (!id) return;

  int slot = findChaseSlot(id);
  if (slot < 0) {
    Serial.println("No chase slots available!");
    return;
  }

  strncpy(chases[slot].id, id, 31);
  strncpy(chases[slot].name, doc["name"] | "Chase", 31);
  chases[slot].bpm = doc["bpm"] | 120;
  chases[slot].loop = doc["loop"] | true;
  chases[slot].stepCount = 0;

  JsonArray steps = doc["steps"];
  if (steps) {
    for (JsonVariant stepVar : steps) {
      if (chases[slot].stepCount >= MAX_CHASE_STEPS) break;

      int stepIdx = chases[slot].stepCount;
      chases[slot].steps[stepIdx].channelCount = 0;

      JsonObject stepChannels = stepVar["channels"];
      if (stepChannels) {
        for (JsonPair kv : stepChannels) {
          if (chases[slot].steps[stepIdx].channelCount >= MAX_STEP_CHANNELS) break;
          int chIdx = chases[slot].steps[stepIdx].channelCount;
          chases[slot].steps[stepIdx].channels[chIdx] = atoi(kv.key().c_str());
          chases[slot].steps[stepIdx].values[chIdx] = kv.value().as<uint8_t>();
          chases[slot].steps[stepIdx].channelCount++;
        }
      }
      chases[slot].stepCount++;
    }
  }

  chases[slot].valid = true;
  Serial.printf("Stored chase '%s' (%d steps, %d BPM)\n",
    chases[slot].name, chases[slot].stepCount, chases[slot].bpm);
}

void playScene(const char* id, int fadeMs) {
  // Stop any running chase
  isPlayingChase = false;
  currentChaseIndex = -1;

  // Find scene
  for (int i = 0; i < MAX_SCENES; i++) {
    if (scenes[i].valid && strcmp(scenes[i].id, id) == 0) {
      int fade = fadeMs >= 0 ? fadeMs : scenes[i].fadeMs;
      for (int j = 0; j < scenes[i].channelCount; j++) {
        startFade(scenes[i].channels[j], scenes[i].values[j], fade);
      }
      Serial.printf("Playing scene '%s' (fade=%dms)\n", scenes[i].name, fade);
      return;
    }
  }
  Serial.printf("Scene '%s' not found\n", id);
}

void playChase(const char* id) {
  for (int i = 0; i < MAX_CHASES; i++) {
    if (chases[i].valid && strcmp(chases[i].id, id) == 0) {
      currentChaseIndex = i;
      currentChaseStep = 0;
      isPlayingChase = true;
      chaseStepInterval = 60000 / chases[i].bpm;
      lastChaseStepTime = millis();
      Serial.printf("Playing chase '%s' (%d BPM)\n", chases[i].name, chases[i].bpm);
      return;
    }
  }
  Serial.printf("Chase '%s' not found\n", id);
}

void stopPlayback() {
  isPlayingChase = false;
  currentChaseIndex = -1;
  Serial.println("Playback stopped");
}

void processChase() {
  if (!isPlayingChase || currentChaseIndex < 0) return;

  unsigned long now = millis();
  if (now - lastChaseStepTime < chaseStepInterval) return;

  StoredChase& chase = chases[currentChaseIndex];
  if (currentChaseStep >= chase.stepCount) {
    if (chase.loop) {
      currentChaseStep = 0;
    } else {
      isPlayingChase = false;
      return;
    }
  }

  // Apply step
  ChaseStep& step = chase.steps[currentChaseStep];
  for (int i = 0; i < step.channelCount; i++) {
    dmxData[step.channels[i]] = step.values[i];
  }

  currentChaseStep++;
  lastChaseStepTime = now;
}

// ═══════════════════════════════════════════════════════════════
// JSON COMMAND HANDLER
// ═══════════════════════════════════════════════════════════════
void handleJsonCommand(const String& jsonStr) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error) {
    Serial.printf("JSON error: %s\n", error.c_str());
    return;
  }

  commandsReceived++;
  const char* cmd = doc["cmd"];
  if (!cmd) return;

  unsigned long fadeMs = doc["fade"] | 0;

  // Extract universe from command (defaults to currentUniverse for backwards compatibility)
  int cmdUniverse = doc["universe"] | currentUniverse;

#ifdef ENABLE_ESPNOW_GATEWAY
  // GATEWAY MODE: Route commands to appropriate universe buffer
  UniverseBuffer* univBuf = getUniverseBuffer(cmdUniverse);

  // Scene data (array) - PRIMARY COMMAND FROM PI
  if (strcmp(cmd, "scene") == 0) {
    int startCh = doc["ch"] | 1;
    JsonArray data = doc["data"];
    if (data && univBuf) {
      int idx = 0;
      for (JsonVariant v : data) {
        int ch = startCh + idx;
        if (ch > 512) break;
        univBuf->data[ch - 1] = v.as<uint8_t>();
        idx++;
      }
      univBuf->dirty = true;
      univBuf->active = true;
      univBuf->lastUpdateTime = millis();
      univBuf->framesReceived++;
      Serial.printf("GW: U%d scene %d ch, fade=%lums\n", cmdUniverse, idx, fadeMs);
    } else if (!univBuf) {
      Serial.printf("GW: Invalid universe %d (max %d)\n", cmdUniverse, MAX_UNIVERSES);
    }

    // Also update local dmxData if this is our local universe (for local DMX output)
    if (cmdUniverse == currentUniverse && data) {
      int startCh2 = doc["ch"] | 1;
      int idx2 = 0;
      for (JsonVariant v : data) {
        int ch = startCh2 + idx2;
        if (ch > 512) break;
        startFade(ch, v.as<uint8_t>(), fadeMs);
        idx2++;
      }
    }
  }
  // Single channel set
  else if (strcmp(cmd, "set") == 0) {
    int ch = doc["ch"];
    int val = doc["val"];
    if (ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
      if (univBuf) {
        univBuf->data[ch - 1] = val;
        univBuf->dirty = true;
        univBuf->active = true;
        univBuf->lastUpdateTime = millis();
      }
      if (cmdUniverse == currentUniverse) {
        startFade(ch, val, fadeMs);
      }
    }
  }
  // Sparse channel updates
  else if (strcmp(cmd, "set_channels") == 0) {
    JsonObject channels = doc["channels"];
    if (channels && univBuf) {
      for (JsonPair kv : channels) {
        int ch = atoi(kv.key().c_str());
        int val = kv.value().as<int>();
        if (ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
          univBuf->data[ch - 1] = val;
          if (cmdUniverse == currentUniverse) {
            startFade(ch, val, fadeMs);
          }
        }
      }
      univBuf->dirty = true;
      univBuf->active = true;
      univBuf->lastUpdateTime = millis();
    }
  }
  // Blackout - affects specified universe (or all if no universe specified)
  else if (strcmp(cmd, "blackout") == 0) {
    stopPlayback();
    bool allUniverses = !doc["universe"].is<int>();  // No universe = blackout all

    for (int u = 1; u <= MAX_UNIVERSES; u++) {
      if (allUniverses || u == cmdUniverse) {
        UniverseBuffer* buf = getUniverseBuffer(u);
        if (buf && buf->active) {
          memset(buf->data, 0, 512);
          buf->dirty = true;
          buf->lastUpdateTime = millis();
          Serial.printf("GW: U%d blackout\n", u);
        }
      }
    }
    // Local blackout
    for (int i = 1; i <= 512; i++) {
      startFade(i, 0, fadeMs);
    }
  }
  // All channels same value
  else if (strcmp(cmd, "all") == 0) {
    int val = doc["val"] | 0;
    if (univBuf) {
      memset(univBuf->data, val, 512);
      univBuf->dirty = true;
      univBuf->active = true;
      univBuf->lastUpdateTime = millis();
    }
    if (cmdUniverse == currentUniverse) {
      for (int i = 1; i <= 512; i++) {
        startFade(i, val, fadeMs);
      }
    }
  }
  // Gateway stats command
  else if (strcmp(cmd, "gw_stats") == 0) {
    Serial.println("═══ GATEWAY UNIVERSE STATS ═══");
    for (int u = 1; u <= MAX_UNIVERSES; u++) {
      UniverseBuffer* buf = getUniverseBuffer(u);
      if (buf && buf->active) {
        Serial.printf("  U%d: rx=%lu tx=%lu seq=%lu last=%lums ago\n",
          u, buf->framesReceived, buf->framesBroadcast, buf->seq,
          millis() - buf->lastUpdateTime);
      }
    }
    Serial.println("═══════════════════════════════");
  }
#else
  // NODE MODE: Apply commands to local dmxData only

  // Single channel set
  if (strcmp(cmd, "set") == 0) {
    int ch = doc["ch"];
    int val = doc["val"];
    if (ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
      startFade(ch, val, fadeMs);
    }
  }
  // Scene data (array)
  else if (strcmp(cmd, "scene") == 0) {
    int startCh = doc["ch"] | 1;
    JsonArray data = doc["data"];
    if (data) {
      int idx = 0;
      for (JsonVariant v : data) {
        int ch = startCh + idx;
        if (ch > 512) break;
        startFade(ch, v.as<uint8_t>(), fadeMs);
        idx++;
      }
    }
  }
  // Sparse channel updates
  else if (strcmp(cmd, "set_channels") == 0) {
    JsonObject channels = doc["channels"];
    if (channels) {
      for (JsonPair kv : channels) {
        int ch = atoi(kv.key().c_str());
        int val = kv.value().as<int>();
        if (ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
          startFade(ch, val, fadeMs);
        }
      }
    }
  }
  // Blackout
  else if (strcmp(cmd, "blackout") == 0) {
    stopPlayback();
    for (int i = 1; i <= 512; i++) {
      startFade(i, 0, fadeMs);
    }
  }
  // All channels same value
  else if (strcmp(cmd, "all") == 0) {
    int val = doc["val"] | 0;
    for (int i = 1; i <= 512; i++) {
      startFade(i, val, fadeMs);
    }
  }
#endif

  // COMMON COMMANDS (both gateway and node modes)
  // Store scene
  if (strcmp(cmd, "store_scene") == 0) {
    storeScene(doc);
  }
  // Store chase
  else if (strcmp(cmd, "store_chase") == 0) {
    storeChase(doc);
  }
  // Play stored scene
  else if (strcmp(cmd, "play_scene") == 0) {
    const char* id = doc["id"];
    int fade = doc["fade_ms"] | -1;
    if (id) playScene(id, fade);
  }
  // Play stored chase
  else if (strcmp(cmd, "play_chase") == 0) {
    const char* id = doc["id"];
    if (id) playChase(id);
  }
  // Stop playback
  else if (strcmp(cmd, "stop") == 0) {
    stopPlayback();
  }
  // Configuration
  else if (strcmp(cmd, "config") == 0) {
    Serial.println(">>> CONFIG COMMAND RECEIVED <<<");
    if (doc["name"].is<const char*>()) {
      nodeName = doc["name"].as<String>();
      Serial.printf("  Name: %s\n", nodeName.c_str());
    }
    if (doc["universe"].is<int>()) {
      changeUniverse(doc["universe"]);
      Serial.printf("  Universe: %d\n", currentUniverse);
    }
    if (doc["channel_start"].is<int>()) {
      channelStart = doc["channel_start"];
      Serial.printf("  Ch Start: %d\n", channelStart);
    }
    if (doc["channel_end"].is<int>()) {
      channelEnd = doc["channel_end"];
      Serial.printf("  Ch End: %d\n", channelEnd);
    }
    isPaired = true;
    saveConfig();
    Serial.println(">>> CONFIG SAVED - isPaired = true <<<");
  }
  // Unpair - reset to unconfigured state
  else if (strcmp(cmd, "unpair") == 0) {
    Serial.println(">>> UNPAIR COMMAND RECEIVED <<<");
    clearConfig();
#ifdef ENABLE_ESPNOW
    // ESP-NOW nodes need to reboot to switch back to WiFi pairing mode
    Serial.println("✓ Node unpaired - rebooting to WiFi pairing mode...");
    delay(1000);
    ESP.restart();
#else
#ifdef ENABLE_SACN
    initSacn();  // Re-init sACN with default universe
#endif
    sendRegistration();  // Re-register as unpaired
    Serial.println("✓ Node unpaired - awaiting configuration");
#endif
  }
  // Identify (flash LED)
  else if (strcmp(cmd, "identify") == 0) {
    for (int i = 0; i < 20; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  // Reboot
  else if (strcmp(cmd, "reboot") == 0) {
    delay(500);
    ESP.restart();
  }
}

// ═══════════════════════════════════════════════════════════════
// OLED DISPLAY - Professional Layout
// Yellow zone: rows 0-15 (header/status)
// Blue zone: rows 16-63 (main content)
// ═══════════════════════════════════════════════════════════════
#define YELLOW_ZONE_HEIGHT 16
#define BLUE_ZONE_START 16

// Animation frame counter
unsigned long displayFrame = 0;

void drawCenteredText(const char* text, int y, int textSize = 1) {
  display.setTextSize(textSize);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, y);
  display.print(text);
}

void drawProgressBar(int x, int y, int width, int height, int percent) {
  display.drawRect(x, y, width, height, SSD1306_WHITE);
  int fillWidth = (width - 2) * percent / 100;
  if (fillWidth > 0) {
    display.fillRect(x + 1, y + 1, fillWidth, height - 2, SSD1306_WHITE);
  }
}

void drawSignalBars(int x, int y, int rssi) {
  // Convert RSSI to 0-4 bars
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

void drawDmxMeter(int x, int y, int width) {
  // Mini DMX activity meter showing first 16 channels
  for (int i = 0; i < 16; i++) {
    int barHeight = map(dmxData[i + 1], 0, 255, 0, 8);
    int barX = x + (i * (width / 16));
    if (barHeight > 0) {
      display.fillRect(barX, y + 8 - barHeight, 2, barHeight, SSD1306_WHITE);
    }
  }
}

void initOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);

  // Try common OLED addresses
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledPresent = true;
    Serial.println("OLED: Found at 0x3C");
  } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    oledPresent = true;
    Serial.println("OLED: Found at 0x3D");
  } else {
    Serial.println("OLED: Not detected (checked 0x3C, 0x3D)");
    return;
  }

  if (oledPresent) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Boot animation
    display.setTextSize(2);
    drawCenteredText("AETHER", 4, 2);
    display.display();
    delay(500);

    display.setTextSize(1);
    drawCenteredText("PULSE", 24, 1);
    display.display();
    delay(300);

    // Loading bar
    for (int i = 0; i <= 100; i += 5) {
      drawProgressBar(14, 40, 100, 8, i);
      display.display();
      delay(30);
    }

    display.clearDisplay();
    drawCenteredText("INITIALIZING...", 28, 1);
    display.display();
  }
}

void updateOLED() {
  if (!oledPresent) return;

  displayFrame++;
  display.clearDisplay();

  // ════════════════════════════════════════════════════════════
  // YELLOW ZONE (0-15): Header with node name and status icons
  // ════════════════════════════════════════════════════════════
  display.setTextSize(1);

  // Node name (left aligned, truncated if needed)
  display.setCursor(0, 0);
  String displayName = nodeName;
  if (displayName.length() > 12) {
    displayName = displayName.substring(0, 11) + ".";
  }
  display.print(displayName);

  // Status icons (right aligned)
  if (operationMode == MODE_WIRELESS) {
    if (WiFi.status() == WL_CONNECTED) {
      drawSignalBars(100, 0, WiFi.RSSI());
    } else {
      // WiFi disconnected - X icon
      display.drawLine(104, 2, 112, 10, SSD1306_WHITE);
      display.drawLine(112, 2, 104, 10, SSD1306_WHITE);
    }
  } else {
    // Wired mode - cable icon
    display.fillRect(100, 4, 12, 4, SSD1306_WHITE);
    display.fillRect(114, 3, 6, 6, SSD1306_WHITE);
  }

  // DMX activity indicator (blinking dot when active)
  if (dmxFramesSent > 0 && (displayFrame % 2 == 0)) {
    display.fillCircle(122, 5, 3, SSD1306_WHITE);
  } else {
    display.drawCircle(122, 5, 3, SSD1306_WHITE);
  }

  // Separator line
  display.drawLine(0, 14, 128, 14, SSD1306_WHITE);

  // ════════════════════════════════════════════════════════════
  // BLUE ZONE (16-63): Main content based on state
  // ════════════════════════════════════════════════════════════

  if (!isPaired) {
    // ──────────────────────────────────────────
    // NOT CONFIGURED STATE
    // ──────────────────────────────────────────
    display.setTextSize(1);
    drawCenteredText("AWAITING CONFIG", BLUE_ZONE_START + 4, 1);

    // Connection status
    display.setCursor(0, BLUE_ZONE_START + 18);
    if (operationMode == MODE_WIRELESS) {
      if (WiFi.status() == WL_CONNECTED) {
        display.print("WiFi: ");
        display.println(WiFi.SSID().substring(0, 12));
        display.print("IP: ");
        display.println(WiFi.localIP().toString());
      } else {
        display.println("WiFi: Connecting...");
        // Animated dots
        for (int i = 0; i < (displayFrame % 4); i++) {
          display.print(".");
        }
      }
    } else {
      display.println("Mode: WIRED");
      display.println("Awaiting Pi data...");
    }

    // Pulsing border animation
    if (displayFrame % 4 < 2) {
      display.drawRect(0, BLUE_ZONE_START, 128, 48, SSD1306_WHITE);
    }

  } else if (isPlayingChase && currentChaseIndex >= 0) {
    // ──────────────────────────────────────────
    // CHASE PLAYING STATE
    // ──────────────────────────────────────────
    display.setTextSize(1);
    display.setCursor(0, BLUE_ZONE_START + 2);
    display.print("CHASE");

    // Chase name (larger)
    display.setTextSize(1);
    display.setCursor(0, BLUE_ZONE_START + 14);
    display.print(chases[currentChaseIndex].name);

    // Step indicator
    display.setCursor(0, BLUE_ZONE_START + 26);
    display.printf("Step %d/%d", currentChaseStep + 1, chases[currentChaseIndex].stepCount);

    // BPM
    display.setCursor(80, BLUE_ZONE_START + 26);
    display.printf("%dBPM", chases[currentChaseIndex].bpm);

    // Step progress bar
    int stepProgress = (currentChaseStep * 100) / max(1, chases[currentChaseIndex].stepCount - 1);
    drawProgressBar(0, BLUE_ZONE_START + 38, 128, 6, stepProgress);

  } else {
    // ──────────────────────────────────────────
    // NORMAL OPERATING STATE
    // ──────────────────────────────────────────

    // Universe display (prominent)
    display.setTextSize(2);
    display.setCursor(0, BLUE_ZONE_START + 2);
    display.print("U");
    display.print(currentUniverse);

    // Mode badge
    display.setTextSize(1);
    display.setCursor(50, BLUE_ZONE_START + 2);
    display.print(operationMode == MODE_WIRED ? "WIRED" : "sACN");

    // Channel range
    display.setCursor(50, BLUE_ZONE_START + 12);
    display.printf("Ch %d-%d", channelStart, channelEnd);

    // Active channel count
    int activeChannels = 0;
    int totalLevel = 0;
    for (int i = 1; i <= 512; i++) {
      if (dmxData[i] > 0) {
        activeChannels++;
        totalLevel += dmxData[i];
      }
    }

    display.setCursor(0, BLUE_ZONE_START + 24);
    display.printf("Active: %d ch", activeChannels);

    // Average level (if any active)
    if (activeChannels > 0) {
      int avgLevel = (totalLevel / activeChannels) * 100 / 255;
      display.setCursor(80, BLUE_ZONE_START + 24);
      display.printf("@%d%%", avgLevel);
    }

    // DMX activity meter
    display.setCursor(0, BLUE_ZONE_START + 34);
    display.print("DMX:");
    drawDmxMeter(28, BLUE_ZONE_START + 34, 96);

    // Connection quality (wireless only)
    if (operationMode == MODE_WIRELESS && WiFi.status() == WL_CONNECTED) {
      display.setCursor(0, BLUE_ZONE_START + 44);
      display.printf("RSSI:%ddBm", WiFi.RSSI());

      // Packets counter
      display.setCursor(70, BLUE_ZONE_START + 44);
      display.printf("P:%lu", sacnPacketsReceived % 100000);
    } else if (operationMode == MODE_WIRED) {
      display.setCursor(0, BLUE_ZONE_START + 44);
      display.printf("Cmds:%lu", commandsReceived);
      display.setCursor(70, BLUE_ZONE_START + 44);
      display.printf("F:%lu", dmxFramesSent % 100000);
    }
  }

  display.display();
}

// ═══════════════════════════════════════════════════════════════
// OTA UPDATE SUPPORT
// ═══════════════════════════════════════════════════════════════
void initOTA() {
  // Set hostname for OTA (makes it easy to find on network)
  ArduinoOTA.setHostname(nodeName.c_str());

  // Optional: Set OTA password (uncomment to enable)
  // ArduinoOTA.setPassword("aether");

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
    Serial.println("🔄 OTA Update starting: " + type);
    // Stop DMX output during update to prevent glitches
    isPlayingChase = false;
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n✅ OTA Update complete! Rebooting...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    // Blink LED during update
    digitalWrite(LED_PIN, (progress / 1000) % 2);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("❌ OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.printf("✓ OTA enabled: %s.local\n", nodeName.c_str());
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);

  memset(dmxData, 0, sizeof(dmxData));
  memset(fades, 0, sizeof(fades));
  memset(scenes, 0, sizeof(scenes));
  memset(chases, 0, sizeof(chases));

#ifdef ENABLE_ESPNOW_GATEWAY
  initUniverseBuffers();
#endif

  // Generate node ID first (needed for boot banner)
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13];
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  nodeId = String("node-") + String(macStr).substring(8);

  // Print firmware identity banner
  printFirmwareIdentity(nodeId.c_str(), currentUniverse);

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // OLED
  initOLED();

  // Load config
  loadConfig();

  // Detect operation mode
  operationMode = detectMode();
  Serial.printf("Mode: %s\n", operationMode == MODE_WIRED ? "WIRED" : "WIRELESS");

  // Initialize based on mode
  if (operationMode == MODE_WIRED) {
    PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, -1);

#ifdef ENABLE_ESPNOW_GATEWAY
    // WIRED mode with ESP-NOW gateway: init WiFi for ESP-NOW only (no connection needed)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // Don't connect to any AP, just enable radio for ESP-NOW

    // Initialize ESP-NOW gateway for rebroadcasting UART commands to ESP-NOW nodes
    if (EspNowGateway::init()) {
      Serial.println("ESP-NOW Gateway: Active (WIRED mode - will rebroadcast UART frames)");
    } else {
      Serial.println("ESP-NOW Gateway: Init failed!");
    }
#endif

  } else {
#ifdef ENABLE_ESPNOW
    if (isPaired) {
      // ESP-NOW node (PAIRED): init transport layer for ESP-NOW reception
      Transport::init(currentUniverse);
      Transport::onFrame([](uint16_t universe, const uint8_t* channels, uint32_t fade_ms, uint32_t seq) {
        // Copy received frame to DMX buffer
        if (fade_ms > 0) {
          // Apply fades
          for (int i = 0; i < 512; i++) {
            if (channels[i] != dmxData[i + 1]) {
              startFade(i + 1, channels[i], fade_ms);
            }
          }
        } else {
          // Instant update
          memcpy(dmxData + 1, channels, 512);
        }
      });
      Serial.printf("ESP-NOW: Listening on channel %d, universe %d\n", ESPNOW_CHANNEL, currentUniverse);
    } else {
      // ESP-NOW node (UNPAIRED): use WiFi for discovery/pairing
      // WiFi was already connected in detectMode(), just start UDP listeners
      if (WiFi.status() == WL_CONNECTED) {
        discoveryUdp.begin(DISCOVERY_PORT);
        configUdp.begin(CONFIG_PORT);
        Serial.printf("PAIRING MODE: UDP listening on discovery=%d, config=%d\n", DISCOVERY_PORT, CONFIG_PORT);
        initOTA();  // Enable OTA in pairing mode too
        sendRegistration();  // Announce to Pi
        Serial.println(">>> Waiting for pairing via WiFi - will switch to ESP-NOW after config <<<");
      } else {
        Serial.println("WARNING: WiFi not connected, cannot pair. Retry on next boot.");
      }
    }

#else
    // sACN/Hybrid node: connect to WiFi
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    if (WiFi.status() == WL_CONNECTED) {
      // Always start UDP listeners (may have connected in detectMode)
      discoveryUdp.begin(DISCOVERY_PORT);
      configUdp.begin(CONFIG_PORT);
      Serial.printf("UDP listening: discovery=%d, config=%d\n", DISCOVERY_PORT, CONFIG_PORT);

#ifdef ENABLE_SACN
      initSacn();
#endif

#ifdef ENABLE_ESPNOW_GATEWAY
      // Initialize ESP-NOW gateway for rebroadcasting to ESP-NOW nodes
      if (EspNowGateway::init()) {
        Serial.println("ESP-NOW Gateway: Active (will rebroadcast sACN)");
      } else {
        Serial.println("ESP-NOW Gateway: Init failed!");
      }
#endif

      initOTA();  // Enable OTA updates in wireless mode
      sendRegistration();
    }
#endif
  }

  // Initialize DMX output
  initDmxOutput();

  Serial.println("═══════════════════════════════════════");
  Serial.println("READY");
  Serial.println("═══════════════════════════════════════\n");

  digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();
  static String piBuffer = "";
  static String serialBuffer = "";

  // Process fades
  processFades();

  // Process chase playback
  processChase();

  // Mode-specific input handling
  if (operationMode == MODE_WIRED) {
    // Read from Pi UART
    while (PiSerial.available()) {
      char c = PiSerial.read();
      if (c == '\n') {
        piBuffer.trim();
        if (piBuffer.length() > 0) {
          handleJsonCommand(piBuffer);
          lastPiData = now;
        }
        piBuffer = "";
      } else {
        piBuffer += c;
      }
    }
  } else {
#ifdef ENABLE_ESPNOW
    if (isPaired) {
      // ESP-NOW node (PAIRED): poll transport for received frames
      Transport::poll();
    } else {
      // ESP-NOW node (UNPAIRED): handle WiFi pairing mode
      ArduinoOTA.handle();

      // Check for UDP config packets (for pairing)
      int packetSize = configUdp.parsePacket();
      if (packetSize > 0) {
        char buffer[2500];
        int len = configUdp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
          buffer[len] = '\0';
          Serial.printf("PAIRING: UDP Config received (%d bytes)\n", len);
          handleJsonCommand(String(buffer));

          // If we just got paired, reboot to switch to ESP-NOW mode
          if (isPaired) {
            Serial.println("\n>>> PAIRED! Rebooting to ESP-NOW mode... <<<\n");
            delay(1000);
            ESP.restart();
          }
        }
      }

      // Heartbeat every 10 seconds
      if (now - lastHeartbeat >= 10000) {
        if (WiFi.status() == WL_CONNECTED) {
          sendHeartbeat();
        }
        lastHeartbeat = now;
      }

      // WiFi reconnect check
      if (now - lastWifiCheck >= 5000) {
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi lost, reconnecting for pairing...");
          WiFi.begin(WIFI_SSID, strlen(WIFI_PASSWORD) > 0 ? WIFI_PASSWORD : nullptr);
        }
        lastWifiCheck = now;
      }
    }
#else
    // sACN/Hybrid node: handle OTA and sACN
    ArduinoOTA.handle();

#ifdef ENABLE_SACN
    // Check sACN packets
    if (e131 != nullptr && !e131->isEmpty()) {
      e131_packet_t packet;
      e131->pull(&packet);

      // Only update if NOT playing a local chase AND no fades active
      // This prevents sACN from stomping on UDP JSON fade commands
      if (!isPlayingChase && !fadeActive) {
        memcpy(dmxData + 1, packet.property_values + 1, 512);
      }
      sacnPacketsReceived++;
      lastSacnReceived = now;
      // ESP-NOW broadcast now happens in the DMX send loop for consistent 40Hz timing
    }
#endif

    // Check for UDP config packets
    // NOTE: Buffer must be large enough for full 512-channel JSON frames (~2100 bytes max)
    int packetSize = configUdp.parsePacket();
    if (packetSize > 0) {
      char buffer[2500];  // Increased from 512 to handle full 512-channel frames
      int len = configUdp.read(buffer, sizeof(buffer) - 1);
      if (len > 0) {
        buffer[len] = '\0';
        Serial.printf("UDP Config received (%d bytes)\n", len);
        handleJsonCommand(String(buffer));
      }
    }

    // Heartbeat every 10 seconds
    if (now - lastHeartbeat >= 10000) {
      if (WiFi.status() == WL_CONNECTED) {
        sendHeartbeat();
      }
      lastHeartbeat = now;
    }

    // WiFi reconnect check
    if (now - lastWifiCheck >= 5000) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost, reconnecting...");
        connectWiFi();
#ifdef ENABLE_SACN
        if (WiFi.status() == WL_CONNECTED) {
          initSacn();
          sendRegistration();
        }
#endif
      }
      lastWifiCheck = now;
    }
#endif  // !ENABLE_ESPNOW (sACN/Hybrid mode)
  }

  // USB Serial for testing (both modes)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        handleJsonCommand(serialBuffer);
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }

  // Send DMX at 40Hz
  if (now - lastDmxSend >= 25) {
    sendDMXFrame();
    lastDmxSend = now;

#ifdef ENABLE_ESPNOW_GATEWAY
    // MULTI-UNIVERSE BROADCAST: Send all active/dirty universes to ESP-NOW nodes
    // Strategy:
    //   - Dirty universes: broadcast immediately (data changed)
    //   - Active universes: broadcast at keepalive rate (2Hz) to maintain sync
    //   - Inactive universes: skip (no data ever received)

    for (int u = 1; u <= MAX_UNIVERSES; u++) {
      UniverseBuffer* buf = getUniverseBuffer(u);
      if (!buf || !buf->active) continue;

      bool shouldBroadcast = false;
      const char* reason = "";

      // Check if dirty (data changed since last broadcast)
      if (buf->dirty) {
        shouldBroadcast = true;
        reason = "dirty";
        buf->dirty = false;
      }
      // Check keepalive interval (broadcast unchanged data periodically)
      else if (now - buf->lastBroadcastTime >= UNIVERSE_KEEPALIVE_MS) {
        shouldBroadcast = true;
        reason = "keepalive";
      }

      if (shouldBroadcast) {
        buf->seq++;
        bool ok = EspNowGateway::broadcastFrame(u, buf->data, 0, buf->seq);
        if (ok) {
          buf->framesBroadcast++;
          buf->lastBroadcastTime = now;
        }
        // Debug: log every 100th frame per universe
        if (buf->framesBroadcast % 100 == 1) {
          Serial.printf("GW: U%d tx=%lu (%s)\n", u, buf->framesBroadcast, reason);
        }
      }
    }
#endif
  }

  // Update OLED every second
  if (now - lastDisplayUpdate >= 1000) {
    updateOLED();
    lastDisplayUpdate = now;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Debug output every 5 seconds
  static unsigned long lastDebug = 0;
  if (now - lastDebug >= 5000) {
    if (operationMode == MODE_WIRED) {
#ifdef ENABLE_ESPNOW_GATEWAY
      // Gateway mode: show multi-universe stats
      Serial.printf("GW Cmds:%lu DMX:%lu | Active universes: ", commandsReceived, dmxFramesSent);
      int activeCount = 0;
      for (int u = 1; u <= MAX_UNIVERSES; u++) {
        UniverseBuffer* buf = getUniverseBuffer(u);
        if (buf && buf->active) {
          if (activeCount > 0) Serial.print(", ");
          Serial.printf("U%d(rx:%lu/tx:%lu)", u, buf->framesReceived, buf->framesBroadcast);
          activeCount++;
        }
      }
      if (activeCount == 0) Serial.print("none");
      Serial.println();
#else
      Serial.printf("WIRED Cmds:%lu DMX:%lu Ch1-3:[%d,%d,%d]\n",
        commandsReceived, dmxFramesSent,
        dmxData[1], dmxData[2], dmxData[3]);
#endif
    } else {
#ifdef ENABLE_ESPNOW
      if (isPaired) {
        Serial.printf("ESP-NOW RX:%lu Drop:%lu DMX:%lu Ch1-3:[%d,%d,%d]\n",
          Transport::getPacketsReceived(), Transport::getPacketsDropped(), dmxFramesSent,
          dmxData[1], dmxData[2], dmxData[3]);
      } else {
        Serial.printf("PAIRING MODE: WiFi=%s IP=%s - Waiting for config...\n",
          WiFi.status() == WL_CONNECTED ? "OK" : "DISC",
          WiFi.localIP().toString().c_str());
      }
#else
      Serial.printf("U%d sACN:%lu DMX:%lu Ch1-3:[%d,%d,%d] Chase:%s\n",
        currentUniverse, sacnPacketsReceived, dmxFramesSent,
        dmxData[1], dmxData[2], dmxData[3],
        isPlayingChase ? "YES" : "no");
#endif
    }
    lastDebug = now;
  }
}
