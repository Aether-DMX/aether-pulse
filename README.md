# AETHER Pulse - DMX Node Firmware

ESP32-based DMX output nodes for the AETHER DMX system.

## Node Types

### Wireless (WiFi + sACN)
- Connects to AETHER Portal via WiFi
- Receives sACN/E1.31 multicast
- OLED display shows status
- Configurable universe via network

### Wired (Serial)
- Connects to Pi via UART (serial)
- Receives JSON commands directly
- Lower latency, more reliable
- No network configuration needed

---

## Hardware Requirements

### Components (per node)
| Component | Part Number | Notes |
|-----------|-------------|-------|
| MCU | ESP32-DevKitC-32 | 38-pin version |
| RS485 Transceiver | MAX485 TTL Module | 5V tolerant |
| OLED Display | SSD1306 0.96" I2C | Wireless only |
| DMX Connector | 5-Pin XLR Female | Panel mount |
| Terminator | 120Ω Resistor | End of DMX line |
| Power | 5V 2A USB | Or barrel jack |

### Wiring

```
ESP32 → MAX485:
  GPIO17 (TX) → DI
  GPIO16 (RX) → RO (wireless) or Pi TX (wired)
  GPIO4       → DE + RE (active high)
  3.3V        → VCC
  GND         → GND

MAX485 → XLR-5F:
  A+   → Pin 3 (Data+)
  B-   → Pin 2 (Data-)
  GND  → Pin 1 (Ground)

ESP32 → OLED (wireless only):
  GPIO21 → SDA
  GPIO22 → SCL
  3.3V   → VCC
  GND    → GND
```

---

## Flashing Firmware

### Prerequisites
1. Install [PlatformIO](https://platformio.org/install)
2. Install USB-to-Serial driver if needed (CP2102/CH340)

### Flash Wireless Node
```bash
cd wireless
pio run -t upload
pio device monitor  # View serial output
```

### Flash Wired Node
```bash
cd wired
pio run -t upload
pio device monitor
```

### Update COM Port
Edit `platformio.ini` and change `upload_port`:
```ini
upload_port = COM11   ; Windows
upload_port = /dev/ttyUSB0  ; Linux
```

---

## Configuration

### Wireless Node
Default network settings in `wireless/src/main.cpp`:
```cpp
const char* WIFI_SSID = "AetherDMX";      // Portal AP mode
const char* WIFI_PASSWORD = "";            // Open network
const char* CONTROLLER_IP = "192.168.50.1"; // Portal IP in AP mode
```

For your home network, change these before flashing.

Universe and channel settings are stored in flash and can be configured via the Portal UI after pairing.

### Wired Node
No configuration needed. Connect GPIO16 (RX) to Pi's TX pin.

---

## LED Status

| Pattern | Meaning |
|---------|---------|
| Solid ON | Booting |
| Slow blink (1Hz) | Normal operation |
| Fast blink | Identify mode (triggered from Portal) |
| OFF | Error or not powered |

---

## Serial Commands (Wired Node)

Send JSON over serial at 115200 baud:

```json
// Set single channel
{"cmd":"set","ch":1,"val":255}

// Set with fade
{"cmd":"set","ch":1,"val":255,"fade":2000}

// Set multiple channels
{"cmd":"set_channels","channels":{"1":255,"2":128,"3":64}}

// Scene (array starting at channel)
{"cmd":"scene","ch":1,"data":[255,128,64,0]}

// Blackout
{"cmd":"blackout"}

// All channels to value
{"cmd":"all","val":255}
```

---

## Troubleshooting

### No DMX Output
1. Check MAX485 wiring (DE/RE must be HIGH)
2. Verify 120Ω terminator at end of DMX chain
3. Check XLR pin assignments (2=Data-, 3=Data+)

### WiFi Won't Connect (Wireless)
1. Verify SSID/password in firmware
2. Check Portal is in AP mode (192.168.50.1)
3. Move closer to Portal

### OLED Not Working
1. Check I2C address (default 0x3C)
2. Verify SDA/SCL connections
3. Some displays use address 0x3D

### Serial Commands Not Working (Wired)
1. Cross TX/RX: Pi TX → ESP32 RX (GPIO16)
2. Common ground between Pi and ESP32
3. Check baud rate is 115200

---

## Version History

- **v7.1-sacn** (Wireless) - Universe stored in flash, dynamic config
- **v5.2** (Wired) - Stable with fade engine, sparse updates

---

## License

MIT License - See LICENSE file
