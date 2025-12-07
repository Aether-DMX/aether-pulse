# AETHER DMX - AI Development Context

## Project Overview

AETHER is a distributed DMX lighting control system built on Raspberry Pi 5 with ESP32 wireless nodes. The system provides natural language lighting control via Claude AI integration, competing with professional lighting controllers costing $5K-$10K.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 5 Controller                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐ │
│  │  React UI   │  │ aether-core │  │   Hardwired ESP32       │ │
│  │  (Frontend) │◄─┤   (Python)  │◄─┤   (UART /dev/serial0)   │ │
│  │  Port 80    │  │  Port 8891  │  │   DMX Universe 1        │ │
│  └─────────────┘  └──────┬──────┘  └─────────────────────────┘ │
│                          │                                       │
│                     WiFi AP: "AetherDMX" (open, no password)    │
│                     IP: 192.168.50.1                             │
└──────────────────────────┼──────────────────────────────────────┘
                           │ UDP Commands (port 8888)
                           │ UDP Discovery (port 9999)
                           ▼
        ┌──────────────────┴──────────────────┐
        │                                      │
   ┌────┴────┐                           ┌────┴────┐
   │ ESP32   │                           │ ESP32   │
   │ Wireless│                           │ Wireless│
   │ Node 1  │                           │ Node 2  │
   │ DMX Out │                           │ DMX Out │
   └─────────┘                           └─────────┘
```

## Hardware Specifications

### Raspberry Pi 5 Controller
- Raspberry Pi 5 with touchscreen
- Runs Raspberry Pi OS (Bookworm)
- Hosts WiFi AP "AetherDMX" (open network, no password)
- AP IP: 192.168.50.1

### ESP32 Nodes (Both Hardwired and Wireless)
- ESP32-DevKitC-32
- MAX485 RS485 Transceiver for DMX output
- SSD1306 OLED Display (128x64, I2C address 0x3C)

### Pin Configuration (ESP32)
```
GPIO 17 → MAX485 DI (DMX TX)
GPIO 16 → MAX485 RO (DMX RX, optional)
GPIO 4  → MAX485 DE + RE tied together (Enable, HIGH = transmit)
GPIO 21 → OLED SDA
GPIO 22 → OLED SCL
GPIO 2  → Built-in LED (status)
```

### MAX485 Wiring
```
ESP32 3.3V  → MAX485 VCC (pin 8)
ESP32 GND   → MAX485 GND (pin 5)
GPIO 17     → MAX485 DI  (pin 4)
GPIO 4      → MAX485 DE  (pin 3) AND RE (pin 2) tied together
MAX485 A    → XLR Pin 3 (DMX Data+)
MAX485 B    → XLR Pin 2 (DMX Data-)
MAX485 GND  → XLR Pin 1 (Ground)
```

## Software Components

### Backend: aether-core.py (Port 8891)
- Single source of truth for all system functionality
- SQLite database for nodes, scenes, chases, settings
- REST API for frontend communication
- WebSocket for real-time updates
- UDP discovery listener (port 9999)
- Serial communication to hardwired ESP32 (/dev/serial0, 115200 baud)
- UDP commands to wireless nodes (port 8888)

**Key API Endpoints:**
- `GET /api/health` - System health check
- `GET /api/nodes` - List all nodes
- `POST /api/nodes/{id}/pair` - Pair a node
- `POST /api/dmx/set` - Set DMX channels: `{"universe":1,"channels":{"1":255},"fade_ms":0}`
- `POST /api/dmx/blackout` - Blackout universe
- `GET /api/scenes` - List scenes
- `POST /api/scenes` - Create scene
- `POST /api/scenes/{id}/play` - Play scene

### Frontend: React App (Port 80)
- Located at /home/ramzt/Aether-DMX/frontend
- Uses Zustand for state management
- API configuration in src/config/api.js
- Stores: dmxStore, nodeStore, uiStore, backgroundStore

### ESP32 Firmware

**Hardwired Node (Working):**
- Location: Embedded in Pi enclosure
- Communication: UART from Pi (/dev/serial0)
- Receives JSON commands: `{"cmd":"scene","ch":1,"data":[255,128,64]}`

**Wireless Node (v6.0):**
- Uses esp_dmx library for reliable DMX output
- Connects to WiFi AP "AetherDMX" (no password)
- Receives UDP commands on port 8888
- Sends registration/heartbeat to port 9999
- Command format: `{"cmd":"set_channels","channels":{"1":255},"fade_ms":0}`

## Communication Protocols

### Pi → Hardwired ESP32 (UART)
```json
{"cmd":"scene","ch":1,"data":[255,128,64],"fade":500}
{"cmd":"blackout","fade":1000}
{"cmd":"set","ch":1,"val":255}
```

### Pi → Wireless ESP32 (UDP port 8888)
```json
{"cmd":"set_channels","channels":{"1":255,"2":128},"fade_ms":500}
{"cmd":"blackout","fade_ms":1000}
{"cmd":"scene","ch":1,"data":[255,128,64],"fade_ms":0}
```

### ESP32 → Pi Discovery (UDP port 9999)
```json
{
  "type": "register",
  "node_id": "node-XXXX",
  "hostname": "AETHER-XXXX",
  "mac": "00:70:07:XX:XX:XX",
  "ip": "192.168.50.XX",
  "version": "6.0-esp_dmx",
  "rssi": -45
}
```

## Development Workflow

### Pi Development (SSH)
```bash
# Edit aether-core.py
sudo nano /home/ramzt/aether-core.py

# Restart service
sudo systemctl restart aether-core

# Check logs
sudo journalctl -u aether-core -f

# Test API
curl -X POST http://localhost:8891/api/dmx/set \
  -H "Content-Type: application/json" \
  -d '{"universe":1,"channels":{"1":255},"fade_ms":0}'
```

### ESP32 Development (VS Code + PlatformIO)
- Use PlatformIO for compilation and upload
- Monitor: 115200 baud
- Upload may require holding BOOT button during "Connecting..."

## Key Libraries

### ESP32 Firmware
- **esp_dmx** (v4.1.0) - Reliable DMX512 output
- **ArduinoJson** (v7.x) - JSON parsing
- **Adafruit_SSD1306** - OLED display
- **WiFi/WiFiUdp** - Network communication

### Pi Backend
- **Flask** - HTTP API server
- **Flask-SocketIO** - WebSocket support
- **pyserial** - UART communication

## DMX Specifications
- Protocol: DMX512-A
- Baud rate: 250000
- Format: 8N2 (8 data bits, no parity, 2 stop bits)
- Break: 176μs minimum
- MAB (Mark After Break): 12μs minimum
- Refresh rate: 40Hz (25ms between frames)
- Channels: 512 per universe

## Troubleshooting

### No DMX Output from ESP32
1. Check GPIO4 (enable pin) is HIGH
2. Verify MAX485 has power (LED on module)
3. Check XLR wiring: A→Pin3, B→Pin2, GND→Pin1
4. Verify esp_dmx library is installed
5. Check serial monitor for error messages

### WiFi Connection Issues
- SSID must be exactly "AetherDMX" (case sensitive)
- No password (open network)
- Pi AP IP is 192.168.50.1

### Frontend Not Updating
- Check aether-core is running: `sudo systemctl status aether-core`
- Verify API responds: `curl http://localhost:8891/api/health`
- Check browser console for errors

## File Locations

### Raspberry Pi
- Backend: `/home/ramzt/aether-core.py`
- Database: `/home/ramzt/aether-core.db`
- Settings: `/home/ramzt/aether-settings.json`
- Frontend: `/home/ramzt/Aether-DMX/frontend/`
- Service: `/etc/systemd/system/aether-core.service`

### ESP32 (PlatformIO Project)
- `platformio.ini` - Build configuration
- `src/main.cpp` - Main firmware code

## Current Status (December 2025)
- ✅ Pi backend (aether-core.py) - Working
- ✅ Hardwired ESP32 node - Working, outputs DMX
- ✅ Frontend React app - Working
- 🔄 Wireless ESP32 nodes - Using esp_dmx library (v6.0)
- ✅ WiFi communication - Working
- ✅ Node discovery - Working

## Notes for AI Agent
1. Always use esp_dmx library for DMX output on ESP32
2. WiFi network is "AetherDMX" with no password
3. API runs on port 8891, not 3000
4. Use GPIO17 for DMX TX, GPIO4 for enable
5. Test pattern on boot: Ch1=255, Ch2=128, Ch3=64
6. Serial monitor baud rate: 115200
