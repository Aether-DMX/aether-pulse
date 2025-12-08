/**
 * AETHER DMX Hybrid Node v1.0
 * Auto-detects wired (UART from Pi) or wireless (sACN/E1.31) mode
 *
 * Features:
 * - Auto-detect: checks for Pi UART connection on boot
 * - Wired mode: receives JSON commands from Pi via UART
 * - Wireless mode: receives sACN/E1.31 + UDP commands
 * - Local scene/chase storage and playback
 * - Hold-last-look when connection lost
 * - Fade engine with smooth transitions
 */

#include <Arduino.h>
#include <esp_dmx.h>
#include <WiFi.h>
#include <WiFiUdp.h>
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
    "\"version\":\"hybrid-1.0\",\"rssi\":%d,\"uptime\":%lu,\"paired\":%s}",
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
    if (doc["name"].is<const char*>()) {
      nodeName = doc["name"].as<String>();
    }
    if (doc["universe"].is<int>()) {
      changeUniverse(doc["universe"]);
    }
    if (doc["channel_start"].is<int>()) {
      channelStart = doc["channel_start"];
    }
    if (doc["channel_end"].is<int>()) {
      channelEnd = doc["channel_end"];
    }
    isPaired = true;
    saveConfig();
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
// OLED DISPLAY
// ═══════════════════════════════════════════════════════════════
void initOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledPresent = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("AETHER HYBRID");
    display.println("v1.0");
    display.display();
  }
}

void updateOLED() {
  if (!oledPresent) return;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  // Mode indicator
  display.print("AETHER ");
  display.println(operationMode == MODE_WIRED ? "[WIRED]" : "[WIFI]");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 14);

  if (operationMode == MODE_WIRELESS) {
    if (WiFi.status() == WL_CONNECTED) {
      display.printf("WiFi: %d dBm\n", WiFi.RSSI());
    } else {
      display.println("WiFi: DISCONNECTED");
    }
    display.printf("Universe: %d\n", currentUniverse);
  } else {
    display.println("Pi UART connected");
    display.printf("Cmds: %lu\n", commandsReceived);
  }

  // Active channels
  int active = 0;
  for (int i = 1; i <= 512; i++) {
    if (dmxData[i] > 0) active++;
  }
  display.printf("Active: %d ch\n", active);

  // Chase status
  if (isPlayingChase && currentChaseIndex >= 0) {
    display.printf("Chase: %s\n", chases[currentChaseIndex].name);
  }

  // First 3 channels
  display.printf("1-3: %d %d %d", dmxData[1], dmxData[2], dmxData[3]);

  display.display();
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
  Serial.println("  AETHER Hybrid Node v1.0");
  Serial.println("  Auto-detect Wired/Wireless");
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
      initSacn();
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
    int packetSize = configUdp.parsePacket();
    if (packetSize > 0) {
      char buffer[512];
      int len = configUdp.read(buffer, sizeof(buffer) - 1);
      if (len > 0) {
        buffer[len] = '\0';
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
