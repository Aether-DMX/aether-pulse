/**
 * AETHER DMX Wireless Node v7.1-sacn
 * Receives sACN/E1.31 from OLA and outputs DMX
 * 
 * NEW in v7.1:
 * - Universe stored in flash (survives reboot)
 * - Accepts config command to change universe
 * - Dynamic sACN re-subscription
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
// OLED CONFIGURATION
// ═══════════════════════════════════════════════════════════════
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledPresent = false;

// ═══════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════
Preferences preferences;
WiFiUDP discoveryUdp;
WiFiUDP configUdp;
ESPAsyncE131* e131 = nullptr;

uint8_t dmxData[DMX_PACKET_SIZE] = {0};
String nodeId = "";
String nodeName = "";

// Configuration (stored in flash)
int currentUniverse = 1;
int channelStart = 1;
int channelEnd = 512;
bool isPaired = false;

// Stats
unsigned long dmxFramesSent = 0;
unsigned long sacnPacketsReceived = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastDmxSend = 0;
unsigned long lastSacnReceived = 0;

// Forward declarations
void sendRegistration();
void initSacn();
void saveConfig();

// ═══════════════════════════════════════════════════════════════
// CONFIGURATION STORAGE
// ═══════════════════════════════════════════════════════════════
void loadConfig() {
  preferences.begin("aether", true);  // read-only
  currentUniverse = preferences.getInt("universe", 1);
  channelStart = preferences.getInt("ch_start", 1);
  channelEnd = preferences.getInt("ch_end", 512);
  isPaired = preferences.getBool("paired", false);
  nodeName = preferences.getString("name", "");
  preferences.end();
  
  if (nodeName.length() == 0) {
    // Generate default name from MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[5];
    sprintf(macStr, "%02X%02X", mac[4], mac[5]);
    nodeName = String("AETHER-") + String(macStr);
  }
  
  Serial.printf("✓ Config loaded: Universe=%d, Ch=%d-%d, Paired=%s\n", 
    currentUniverse, channelStart, channelEnd, isPaired ? "Yes" : "No");
}

void saveConfig() {
  preferences.begin("aether", false);  // read-write
  preferences.putInt("universe", currentUniverse);
  preferences.putInt("ch_start", channelStart);
  preferences.putInt("ch_end", channelEnd);
  preferences.putBool("paired", isPaired);
  preferences.putString("name", nodeName);
  preferences.end();
  
  Serial.printf("✓ Config saved: Universe=%d, Ch=%d-%d, Paired=%s\n", 
    currentUniverse, channelStart, channelEnd, isPaired ? "Yes" : "No");
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
  
  Serial.println("✓ Config cleared - reset to defaults");
}

// ═══════════════════════════════════════════════════════════════
// sACN MANAGEMENT
// ═══════════════════════════════════════════════════════════════
void initSacn() {
  // Clean up existing
  if (e131 != nullptr) {
    delete e131;
    e131 = nullptr;
  }
  
  // Create new instance
  e131 = new ESPAsyncE131(1);
  
  if (e131->begin(E131_MULTICAST, currentUniverse, 1)) {
    Serial.printf("✓ sACN listening on universe %d\n", currentUniverse);
  } else {
    Serial.println("✗ sACN init failed!");
  }
}

void changeUniverse(int newUniverse) {
  if (newUniverse < 1 || newUniverse > 63999) {
    Serial.printf("✗ Invalid universe: %d\n", newUniverse);
    return;
  }
  
  if (newUniverse == currentUniverse) {
    Serial.printf("Universe already set to %d\n", newUniverse);
    return;
  }
  
  Serial.printf("Changing universe: %d -> %d\n", currentUniverse, newUniverse);
  currentUniverse = newUniverse;
  
  // Clear DMX buffer when changing universe
  memset(dmxData, 0, sizeof(dmxData));
  
  // Re-initialize sACN with new universe
  initSacn();
  
  // Save to flash
  saveConfig();
  
  // Send registration with new universe
  sendRegistration();
}

// ═══════════════════════════════════════════════════════════════
// NETWORK
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
    Serial.println("\n✓ WiFi connected!");
    Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
    discoveryUdp.begin(DISCOVERY_PORT);
    configUdp.begin(CONFIG_PORT);
  } else {
    Serial.println("\n✗ WiFi FAILED!");
  }
}

void sendRegistration() {
  char json[512];
  snprintf(json, sizeof(json),
    "{\"type\":\"register\",\"node_id\":\"%s\",\"hostname\":\"%s\","
    "\"mac\":\"%s\",\"ip\":\"%s\",\"universe\":%d,"
    "\"startChannel\":%d,\"channelCount\":%d,"
    "\"version\":\"7.1-sacn\",\"rssi\":%d,\"uptime\":%lu,\"paired\":%s}",
    nodeId.c_str(), nodeName.c_str(),
    WiFi.macAddress().c_str(), WiFi.localIP().toString().c_str(),
    currentUniverse, channelStart, channelEnd - channelStart + 1,
    WiFi.RSSI(), millis() / 1000, isPaired ? "true" : "false");
  
  discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
  discoveryUdp.print(json);
  discoveryUdp.endPacket();
  Serial.println("📤 Registration sent");
}

void sendHeartbeat() {
  char json[256];
  snprintf(json, sizeof(json),
    "{\"type\":\"heartbeat\",\"node_id\":\"%s\",\"rssi\":%d,"
    "\"uptime\":%lu,\"sacn_pkts\":%lu,\"dmx_frames\":%lu,"
    "\"universe\":%d,\"fps\":%.1f}",
    nodeId.c_str(), WiFi.RSSI(), millis() / 1000,
    sacnPacketsReceived, dmxFramesSent, currentUniverse,
    dmxFramesSent > 0 ? (float)dmxFramesSent / (millis() / 1000.0) : 0);
  
  discoveryUdp.beginPacket(CONTROLLER_IP, DISCOVERY_PORT);
  discoveryUdp.print(json);
  discoveryUdp.endPacket();
}

// ═══════════════════════════════════════════════════════════════
// CONFIG COMMAND HANDLER
// ═══════════════════════════════════════════════════════════════
void handleConfigPacket() {
  int packetSize = configUdp.parsePacket();
  if (packetSize == 0) return;
  
  char buffer[512];
  int len = configUdp.read(buffer, sizeof(buffer) - 1);
  if (len <= 0) return;
  buffer[len] = '\0';
  
  Serial.printf("📥 Config received: %s\n", buffer);
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, buffer);
  
  if (error) {
    Serial.printf("JSON error: %s\n", error.c_str());
    return;
  }
  
  const char* cmd = doc["cmd"];
  if (!cmd) return;
  
  if (strcmp(cmd, "config") == 0) {
    // Full configuration update
    bool changed = false;
    
    if (doc["name"].is<const char*>()) {
      nodeName = doc["name"].as<String>();
      changed = true;
    }
    
    if (doc["universe"].is<int>()) {
      int newUniverse = doc["universe"];
      if (newUniverse != currentUniverse) {
        changeUniverse(newUniverse);
        changed = true;
      }
    }
    
    if (doc["channel_start"].is<int>()) {
      channelStart = doc["channel_start"];
      changed = true;
    }
    
    if (doc["channel_end"].is<int>()) {
      channelEnd = doc["channel_end"];
      changed = true;
    }
    
    isPaired = true;
    
    if (changed) {
      saveConfig();
      sendRegistration();
    }
    
    Serial.printf("✓ Config applied: Universe=%d, Ch=%d-%d, Name=%s\n",
      currentUniverse, channelStart, channelEnd, nodeName.c_str());
  }
  else if (strcmp(cmd, "unpair") == 0) {
    // Unpair and reset to defaults
    clearConfig();
    initSacn();
    sendRegistration();
    Serial.println("✓ Node unpaired");
  }
  else if (strcmp(cmd, "identify") == 0) {
    // Flash LED rapidly for identification
    Serial.println("🔦 Identify requested");
    for (int i = 0; i < 20; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  else if (strcmp(cmd, "reboot") == 0) {
    Serial.println("🔄 Rebooting...");
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
    display.println("AETHER WIRELESS");
    display.println("v7.1-sacn");
    display.display();
    Serial.println("✓ OLED initialized");
  } else {
    Serial.println("⚠ OLED not found");
  }
}

void updateOLED() {
  if (!oledPresent) return;
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  
  // Header with name
  display.println(nodeName.length() > 16 ? nodeName.substring(0, 16) : nodeName);
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(0, 14);
  if (WiFi.status() == WL_CONNECTED) {
    display.printf("WiFi: %d dBm\n", WiFi.RSSI());
  } else {
    display.println("WiFi: DISCONNECTED");
  }
  
  // Universe and channel range
  display.printf("Universe: %d\n", currentUniverse);
  display.printf("Channels: %d-%d\n", channelStart, channelEnd);
  
  // Stats
  display.printf("sACN: %lu pkts\n", sacnPacketsReceived);
  
  // First 3 channels
  display.printf("1-3: %d %d %d", dmxData[1], dmxData[2], dmxData[3]);
  
  // Paired indicator
  if (isPaired) {
    display.setCursor(110, 0);
    display.print("P");
  }
  
  display.display();
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  
  // Zero DMX buffer immediately
  memset(dmxData, 0, sizeof(dmxData));
  
  Serial.println("\n");
  Serial.println("═══════════════════════════════════════");
  Serial.println("  AETHER Wireless Node v7.1-sacn");
  Serial.println("  sACN/E1.31 Receiver + Config");
  Serial.println("═══════════════════════════════════════");
  
  // Generate node ID from MAC
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13];
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  nodeId = String("node-") + String(macStr).substring(8);
  Serial.printf("Node ID: %s\n", nodeId.c_str());
  
  // Load configuration from flash
  loadConfig();
  
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize DMX using esp_dmx library
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(DMX_PORT, &config, NULL, 0);
  dmx_set_pin(DMX_PORT, DMX_TX_PIN, DMX_RX_PIN, DMX_ENABLE_PIN);
  
  Serial.println("✓ esp_dmx initialized");
  Serial.printf("  TX: GPIO%d, EN: GPIO%d\n", DMX_TX_PIN, DMX_ENABLE_PIN);
  
  // OLED
  initOLED();
  
  // WiFi
  connectWiFi();
  
  // Initialize sACN/E1.31 receiver
  if (WiFi.status() == WL_CONNECTED) {
    initSacn();
    sendRegistration();
  }
  
  Serial.println("═══════════════════════════════════════");
  Serial.printf("✓ READY - Listening for sACN on universe %d\n", currentUniverse);
  Serial.println("═══════════════════════════════════════\n");
  
  digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();
  
  // Check for config packets
  handleConfigPacket();
  
  // Check for sACN packets
  if (e131 != nullptr && !e131->isEmpty()) {
    e131_packet_t packet;
    e131->pull(&packet);
    memcpy(dmxData + 1, packet.property_values + 1, 512);
    sacnPacketsReceived++;
    lastSacnReceived = now;
  }
  
  // Send DMX at ~40Hz
  if (now - lastDmxSend >= 25) {
    dmx_write(DMX_PORT, dmxData, DMX_PACKET_SIZE);
    dmx_send(DMX_PORT);
    dmx_wait_sent(DMX_PORT, DMX_TIMEOUT_TICK);
    dmxFramesSent++;
    lastDmxSend = now;
    
    // Debug every 5 seconds
    static unsigned long lastDebug = 0;
    if (now - lastDebug >= 5000) {
      Serial.printf("U%d sACN: %lu pkts, DMX: %lu frames, Ch1-3: [%d,%d,%d]\n", 
        currentUniverse, sacnPacketsReceived, dmxFramesSent, 
        dmxData[1], dmxData[2], dmxData[3]);
      lastDebug = now;
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
      Serial.println("⚠ WiFi lost, reconnecting...");
      connectWiFi();
      if (WiFi.status() == WL_CONNECTED) {
        initSacn();
        sendRegistration();
      }
    }
    lastWifiCheck = now;
  }
  
  // Update OLED every second
  if (now - lastDisplayUpdate >= 1000) {
    updateOLED();
    lastDisplayUpdate = now;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
