# AETHER Pulse ESP-NOW DMX Streaming

High-performance, low-latency DMX universe streaming over ESP-NOW for the AETHER DMX ecosystem.

## Overview

This implementation provides two firmware variants:

1. **Gateway** - Connects to Raspberry Pi via USB/Serial, broadcasts DMX frames over ESP-NOW
2. **Node** - Receives ESP-NOW packets, outputs DMX512 with proper timing

## Architecture

```
┌─────────────┐     Serial      ┌─────────────────┐   ESP-NOW    ┌─────────────────┐
│  Pi Portal  │────────────────▶│  ESP-NOW        │─────────────▶│  ESP-NOW Node   │──▶ DMX
│  (aether)   │  Framed Proto   │  Gateway        │  Broadcast   │  (Universe 1)   │
└─────────────┘                 └─────────────────┘              └─────────────────┘
                                        │                               
                                        │ ESP-NOW                ┌─────────────────┐
                                        └───────────────────────▶│  ESP-NOW Node   │──▶ DMX
                                                                 │  (Universe 2)   │
                                                                 └─────────────────┘
```

## Features

- **Low Latency**: Direct ESP-NOW broadcast, ~5-10ms typical
- **Reliable**: Sequence numbers, CRC16 validation, missing chunk detection
- **Hold-Last-Look**: Nodes maintain output on signal loss
- **Test Mode**: Built-in test patterns when no serial connected
- **Multi-Universe**: Each node subscribes to specific universe(s)
- **Diagnostics**: OLED display, serial logging, heartbeat stats

## Hardware Requirements

### Gateway
- ESP32-DevKitC or similar
- USB connection to Raspberry Pi

### Node
- ESP32-DevKitC or similar
- MAX485/MAX3485 RS485 transceiver
- DMX XLR connector
- Optional: SSD1306 OLED (128x64)

### Pin Configuration

| Function     | Gateway | Node   |
|--------------|---------|--------|
| DMX TX       | -       | GPIO17 |
| DMX RX       | -       | GPIO16 |
| DMX Enable   | -       | GPIO4  |
| LED          | GPIO2   | GPIO2  |
| OLED SDA     | GPIO21  | GPIO21 |
| OLED SCL     | GPIO22  | GPIO22 |

## Build Instructions

### PlatformIO (Recommended)

```bash
# Build Gateway
pio run -e espnow_gateway

# Build Node
pio run -e espnow_node

# Upload Gateway
pio run -e espnow_gateway -t upload

# Upload Node  
pio run -e espnow_node -t upload

# Monitor
pio device monitor -b 115200
```

### Arduino IDE

1. Open `gateway/gateway.ino` or `node/node.ino`
2. Select "ESP32 Dev Module"
3. Set partition scheme to "Default 4MB with spiffs"
4. Upload

## Configuration

### Wi-Fi Channel
Both gateway and nodes must use the same Wi-Fi channel. Default is channel 6.

Edit in `common/config.h`:
```cpp
#define ESPNOW_CHANNEL 6
```

### Universe Assignment
Nodes subscribe to specific universe(s). Configure via serial command:
```
{"cmd":"config","universe":1}
```

Or edit in `common/config.h`:
```cpp
#define DEFAULT_UNIVERSE 1
```

## Serial Protocol (Pi → Gateway)

The gateway expects framed DMX data from the Pi:

```
┌──────────┬──────────┬──────────┬────────┬─────────┬───────┐
│ Preamble │ Universe │ Sequence │ Length │ Payload │ CRC16 │
│  2 bytes │  2 bytes │  4 bytes │ 2 bytes│ N bytes │ 2 bytes│
│  0xAA55  │  uint16  │  uint32  │ uint16 │ DMX data│ CRC16 │
└──────────┴──────────┴──────────┴────────┴─────────┴───────┘
```

Example Python sender (for Pi):
```python
import struct
import serial

def send_dmx_frame(ser, universe, seq, dmx_data):
    payload = bytes(dmx_data[:512]).ljust(512, b'\x00')
    header = struct.pack('<HHLH', 0x55AA, universe, seq, 512)
    frame = header + payload
    crc = crc16_ccitt(frame)
    ser.write(b'\xAA\x55' + frame + struct.pack('<H', crc))
```

## ESP-NOW Packet Format

DMX512 (512 bytes) is chunked into ESP-NOW packets (~200 bytes each):

```cpp
struct DmxNowPacket {
    uint8_t  magic[2];      // 'A','X' (0x41, 0x58)
    uint8_t  version;       // Protocol version (1)
    uint8_t  role;          // 1=gateway, 2=node
    uint16_t universe;      // DMX universe ID
    uint32_t seq;           // Frame sequence number
    uint8_t  chunk_index;   // Chunk index (0-2)
    uint8_t  chunk_total;   // Total chunks (3)
    uint16_t data_len;      // Bytes in this chunk
    uint8_t  data[200];     // DMX channel data
    uint16_t crc16;         // CRC16-CCITT
};
```

## Test Mode

When no serial data is received, the gateway enters test mode and generates patterns:

- **Sine Wave**: Channels 1-16 cycle through sine wave
- **Chase**: Sequential channel activation
- **Full White**: All channels at 255

Enable in `common/config.h`:
```cpp
#define TEST_PATTERN_ENABLED 1
#define TEST_PATTERN_MODE TEST_PATTERN_SINE
```

## Monitoring & Debugging

### Gateway Output
```
[GW] Serial frame: U1 seq=1234 len=512 CRC=OK
[GW] ESP-NOW sent: chunk 0/3 -> broadcast
[GW] ESP-NOW sent: chunk 1/3 -> broadcast
[GW] ESP-NOW sent: chunk 2/3 -> broadcast
[GW] Stats: 40 fps, 0 drops, 0 CRC errors
```

### Node Output
```
[NODE] ESP-NOW recv: U1 seq=1234 chunk=0/3
[NODE] ESP-NOW recv: U1 seq=1234 chunk=1/3
[NODE] ESP-NOW recv: U1 seq=1234 chunk=2/3
[NODE] Frame complete: U1 seq=1234
[NODE] DMX out: 40 Hz, Ch1-3=[255,128,64]
```

## Smoke Test Procedure

1. **Flash Gateway**: Upload `espnow_gateway` to ESP32 connected to PC
2. **Flash Node**: Upload `espnow_node` to ESP32 with DMX output
3. **Open Serial Monitors**: Both devices at 115200 baud
4. **Verify Gateway Test Mode**: Should see test pattern being sent
5. **Verify Node Reception**: Should see packets received, DMX output active
6. **Connect DMX Fixture**: Verify light responds to test pattern

## File Structure

```
esp-now/
├── README.md
├── platformio.ini
├── common/
│   ├── config.h          # Shared configuration
│   ├── packet.h          # Packet structures
│   ├── crc16.h           # CRC16-CCITT implementation
│   └── crc16.cpp
├── gateway/
│   ├── gateway.ino       # Arduino IDE entry
│   └── src/
│       ├── main.cpp      # PlatformIO entry
│       ├── serial_rx.h   # Serial frame receiver
│       ├── serial_rx.cpp
│       ├── espnow_tx.h   # ESP-NOW transmitter
│       ├── espnow_tx.cpp
│       ├── test_pattern.h
│       └── test_pattern.cpp
└── node/
    ├── node.ino          # Arduino IDE entry
    └── src/
        ├── main.cpp      # PlatformIO entry
        ├── espnow_rx.h   # ESP-NOW receiver
        ├── espnow_rx.cpp
        ├── dmx_output.h  # DMX512 output
        ├── dmx_output.cpp
        ├── frame_buffer.h
        └── frame_buffer.cpp
```

## License

Part of the AETHER DMX project. MIT License.
