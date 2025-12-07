// ESP32 AETHER Hardwired Node v5.2
// Fixed DMX timing + reliable operation + sparse channel updates

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════
#define DMX_TX_PIN 17
#define DMX_ENABLE_PIN 4
#define PI_RX_PIN 16
#define LED_PIN 2
#define OLED_SDA 21
#define OLED_SCL 22

// ═══════════════════════════════════════════════════════════════
// DMX CONSTANTS - CRITICAL TIMING VALUES
// ═══════════════════════════════════════════════════════════════
#define DMX_BREAK_US 176      // BREAK: minimum 88μs, standard 176μs
#define DMX_MAB_US 12         // Mark After Break: minimum 8μs, standard 12μs
#define DMX_PACKET_SIZE 513   // Start code + 512 channels
#define DMX_REFRESH_HZ 40     // 40Hz = 25ms between frames

// ═══════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════
HardwareSerial DmxSerial(2);  // UART2 for DMX output
HardwareSerial PiSerial(1);   // UART1 for Pi commands

uint8_t dmxBuffer[DMX_PACKET_SIZE] = {0};

// Fade engine
struct Fade {
  uint8_t start;
  uint8_t target;
  unsigned long startTime;
  unsigned long duration;
  bool active;
} fades[DMX_PACKET_SIZE];

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledPresent = false;

// Stats
unsigned long dmxFramesSent = 0;
unsigned long commandsReceived = 0;
unsigned long lastStatsTime = 0;

// Function declarations
void sendDMXFrame();
void processFades();
void startFade(int channel, uint8_t targetValue, unsigned long durationMs);
void handleJsonCommand(const String& jsonStr);
void initOLED();
void updateOLED();

// ═══════════════════════════════════════════════════════════════
// DMX OUTPUT - PROPER TIMING
// ═══════════════════════════════════════════════════════════════

void setup() {
  // Debug serial (USB)
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n");
  Serial.println("═══════════════════════════════════════");
  Serial.println("  AETHER HARDWIRED NODE v5.2");
  Serial.println("  Software DMX with proper timing");
  Serial.println("═══════════════════════════════════════");
  
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // DMX Enable - ALWAYS HIGH for transmit mode
  pinMode(DMX_ENABLE_PIN, OUTPUT);
  digitalWrite(DMX_ENABLE_PIN, HIGH);
  Serial.println("✓ DMX enable pin HIGH");
  
  // Initialize DMX serial
  DmxSerial.begin(250000, SERIAL_8N2, -1, DMX_TX_PIN);
  Serial.println("✓ DMX serial initialized (250kbaud)");
  
  // Initialize Pi command serial
  PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, -1);
  Serial.println("✓ Pi serial ready (GPIO16)");
  
  // Initialize DMX buffer (start code = 0)
  memset(dmxBuffer, 0, DMX_PACKET_SIZE);
  memset(fades, 0, sizeof(fades));
  
  // OLED
  initOLED();
  
  // Send initial DMX frame
  sendDMXFrame();
  
  Serial.println("═══════════════════════════════════════");
  Serial.println("✓ READY - Listening for JSON commands");
  Serial.println("═══════════════════════════════════════\n");
  
  digitalWrite(LED_PIN, LOW);
}

// ═══════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════

void loop() {
  static unsigned long lastDmxTime = 0;
  static unsigned long lastOledTime = 0;
  static unsigned long lastBlinkTime = 0;
  static String serialBuffer = "";
  static String piBuffer = "";
  
  unsigned long now = millis();
  
  // Process fades
  processFades();
  
  // Send DMX at 40Hz (every 25ms)
  if (now - lastDmxTime >= 25) {
    sendDMXFrame();
    lastDmxTime = now;
  }
  
  // Read from USB Serial (for testing)
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
  
  // Read from Pi Serial
  while (PiSerial.available()) {
    char c = PiSerial.read();
    if (c == '\n') {
      piBuffer.trim();
      if (piBuffer.length() > 0) {
        handleJsonCommand(piBuffer);
      }
      piBuffer = "";
    } else {
      piBuffer += c;
    }
  }
  
  // Update OLED every second
  if (now - lastOledTime >= 1000) {
    updateOLED();
    lastOledTime = now;
  }
  
  // LED heartbeat
  if (now - lastBlinkTime >= 500) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlinkTime = now;
  }
  
  // Stats every 10 seconds
  if (now - lastStatsTime >= 10000) {
    Serial.printf("Stats: DMX frames=%lu, Commands=%lu, Ch1-3=[%d,%d,%d]\n",
      dmxFramesSent, commandsReceived,
      dmxBuffer[1], dmxBuffer[2], dmxBuffer[3]);
    lastStatsTime = now;
  }
}

void initOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledPresent = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("AETHER HARDWIRED");
    display.println("v5.2");
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
  
  display.println("AETHER HARDWIRED");
  display.println("────────────────");
  
  // Active channels
  int active = 0;
  for (int i = 1; i <= 512; i++) {
    if (dmxBuffer[i] > 0) active++;
  }
  display.printf("Active: %d ch\n", active);
  
  // Frame rate
  display.printf("DMX: 40 Hz\n");
  
  // Commands received
  display.printf("Cmds: %lu\n", commandsReceived);
  
  // Show first 3 channels
  display.printf("Ch1-3: %d %d %d\n", 
    dmxBuffer[1], dmxBuffer[2], dmxBuffer[3]);
  
  display.display();
}

// ═══════════════════════════════════════════════════════════════
// DMX OUTPUT FUNCTION
// ═══════════════════════════════════════════════════════════════
void sendDMXFrame() {
  // Disable UART to manually control pin for BREAK
  DmxSerial.end();
  
  // BREAK: Hold line LOW for 176μs (DMX512 spec: min 88μs)
  pinMode(DMX_TX_PIN, OUTPUT);
  digitalWrite(DMX_TX_PIN, LOW);
  delayMicroseconds(DMX_BREAK_US);
  
  // MAB (Mark After Break): Hold HIGH for 12μs (spec: min 8μs)
  digitalWrite(DMX_TX_PIN, HIGH);
  delayMicroseconds(DMX_MAB_US);
  
  // Re-enable UART and send packet
  DmxSerial.begin(250000, SERIAL_8N2, -1, DMX_TX_PIN);
  
  // Send start code (0x00) + 512 channels
  DmxSerial.write(dmxBuffer, DMX_PACKET_SIZE);
  DmxSerial.flush();  // Wait for transmission to complete
  
  dmxFramesSent++;
}

// ═══════════════════════════════════════════════════════════════
// FADE ENGINE
// ═══════════════════════════════════════════════════════════════

void processFades() {
  unsigned long now = millis();
  
  for (int i = 1; i <= 512; i++) {
    if (!fades[i].active) continue;
    
    unsigned long elapsed = now - fades[i].startTime;
    
    if (elapsed >= fades[i].duration) {
      dmxBuffer[i] = fades[i].target;
      fades[i].active = false;
    } else {
      float progress = (float)elapsed / fades[i].duration;
      // Sine ease-in-out for smooth fades
      float eased = 0.5f - 0.5f * cosf(progress * PI);
      int range = fades[i].target - fades[i].start;
      dmxBuffer[i] = fades[i].start + (uint8_t)(range * eased);
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// JSON COMMAND PARSER
// ═══════════════════════════════════════════════════════════════

void startFade(int channel, uint8_t targetValue, unsigned long durationMs) {
  if (channel < 1 || channel > 512) return;
  
  if (durationMs == 0) {
    dmxBuffer[channel] = targetValue;
    fades[channel].active = false;
  } else {
    fades[channel].start = dmxBuffer[channel];
    fades[channel].target = targetValue;
    fades[channel].startTime = millis();
    fades[channel].duration = durationMs;
    fades[channel].active = true;
  }
}

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
  
  if (strcmp(cmd, "set") == 0) {
    int ch = doc["ch"];
    int val = doc["val"];
    if (ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
      startFade(ch, val, fadeMs);
      Serial.printf("SET ch%d=%d fade=%lums\n", ch, val, fadeMs);
    }
  }
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
      Serial.printf("SCENE ch%d count=%d fade=%lums\n", startCh, idx, fadeMs);
    }
  }
  // NEW: Sparse channel updates - only send changed channels
  else if (strcmp(cmd, "set_channels") == 0) {
    JsonObject channels = doc["channels"];
    if (channels) {
      int count = 0;
      for (JsonPair kv : channels) {
        int ch = atoi(kv.key().c_str());
        int val = kv.value().as<int>();
        if (ch >= 1 && ch <= 512 && val >= 0 && val <= 255) {
          startFade(ch, val, fadeMs);
          count++;
        }
      }
      Serial.printf("SET_CHANNELS count=%d fade=%lums\n", count, fadeMs);
    }
  }
  else if (strcmp(cmd, "blackout") == 0) {
    for (int i = 1; i <= 512; i++) {
      startFade(i, 0, fadeMs);
    }
    Serial.println("BLACKOUT");
  }
  else if (strcmp(cmd, "all") == 0) {
    int val = doc["val"] | 0;
    for (int i = 1; i <= 512; i++) {
      startFade(i, val, fadeMs);
    }
    Serial.printf("ALL=%d fade=%lums\n", val, fadeMs);
  }
}
