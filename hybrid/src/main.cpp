/**
 * AETHER DMX Hybrid Node v1.2
 * Auto-detects wired (UART from Pi) or wireless (sACN/E1.31) mode
 *
 * Features:
 * - Auto-detect: checks for Pi UART connection on boot
 * - Wired mode: receives JSON commands from Pi via UART
 * - Wireless mode: receives sACN/E1.31 + UDP commands
 * - Local scene/chase storage and playback
 * - Hold-last-look when connection lost
 * - Fade engine with smooth transitions
 * - OTA updates (wireless mode only)
 */

#include <Arduino.h>
#include <esp_dmx.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESPAsyncE131.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════
#define DMX_TX_PIN 17
#define DMX_RX_PIN 16
#define DMX_ENABLE_PIN 4
#define DMX_PORT 1

#define PI_RX_PIN 16      // Same as DMX_RX - reused in wired mode
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
ESPAsyncE131* e131 = nullptr;
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
void initSacn();
void saveConfig();
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

// ═══════════════════════════════════════════════════════════════
// sACN MANAGEMENT (Wireless mode)
// ═══════════════════════════════════════════════════════════════
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

void changeUniverse(int newUniverse) {
  if (newUniverse < 1 || newUniverse > 63999) return;
  if (newUniverse == currentUniverse) return;

  Serial.printf("Universe: %d -> %d\n", currentUniverse, newUniverse);
  currentUniverse = newUniverse;

  if (operationMode == MODE_WIRELESS) {
    initSacn();
  }

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
    "\"version\":\"hybrid-1.2\",\"rssi\":%d,\"uptime\":%lu,\"paired\":%s}",
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

  for (int i = 1; i <= 512; i++) {
    if (!fades[i].active) continue;

    unsigned long elapsed = now - fades[i].startTime;

    if (elapsed >= fades[i].duration) {
      dmxData[i] = fades[i].target;
      fades[i].active = false;
    } else {
      float progress = (float)elapsed / fades[i].duration;
      float eased = 0.5f - 0.5f * cosf(progress * PI);
      int range = fades[i].target - fades[i].start;
      dmxData[i] = fades[i].start + (uint8_t)(range * eased);
    }
  }
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
  // Store scene
  else if (strcmp(cmd, "store_scene") == 0) {
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
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledPresent = true;
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

  Serial.println("\n");
  Serial.println("═══════════════════════════════════════");
  Serial.println("  AETHER Hybrid Node v1.2");
  Serial.println("  Auto-detect Wired/Wireless + OTA");
  Serial.println("═══════════════════════════════════════");

  // Generate node ID
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13];
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  nodeId = String("node-") + String(macStr).substring(8);
  Serial.printf("Node ID: %s\n", nodeId.c_str());

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
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    if (WiFi.status() == WL_CONNECTED) {
      // Always start UDP listeners (may have connected in detectMode)
      discoveryUdp.begin(DISCOVERY_PORT);
      configUdp.begin(CONFIG_PORT);
      Serial.printf("UDP listening: discovery=%d, config=%d\n", DISCOVERY_PORT, CONFIG_PORT);

      initSacn();
      initOTA();  // Enable OTA updates in wireless mode
      sendRegistration();
    }
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

  // Handle OTA updates (wireless mode only)
  if (operationMode == MODE_WIRELESS) {
    ArduinoOTA.handle();
  }

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
    // Wireless mode: check sACN packets
    if (e131 != nullptr && !e131->isEmpty()) {
      e131_packet_t packet;
      e131->pull(&packet);

      // Only update if NOT playing a local chase
      if (!isPlayingChase) {
        memcpy(dmxData + 1, packet.property_values + 1, 512);
      }
      sacnPacketsReceived++;
      lastSacnReceived = now;
    }

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
        if (WiFi.status() == WL_CONNECTED) {
          initSacn();
          sendRegistration();
        }
      }
      lastWifiCheck = now;
    }
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
    if (operationMode == MODE_WIRELESS) {
      Serial.printf("U%d sACN:%lu DMX:%lu Ch1-3:[%d,%d,%d] Chase:%s\n",
        currentUniverse, sacnPacketsReceived, dmxFramesSent,
        dmxData[1], dmxData[2], dmxData[3],
        isPlayingChase ? "YES" : "no");
    } else {
      Serial.printf("WIRED Cmds:%lu DMX:%lu Ch1-3:[%d,%d,%d]\n",
        commandsReceived, dmxFramesSent,
        dmxData[1], dmxData[2], dmxData[3]);
    }
    lastDebug = now;
  }
}
