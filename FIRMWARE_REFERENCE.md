# AETHER Pulse Node Firmware Reference

**Version:** 2.3.0
**Platform:** ESP32 (PlatformIO)
**Transport:** sACN/E1.31 (Multicast)

---

## Overview

The AETHER Pulse is an ESP32-based DMX node that:
1. Receives sACN/E1.31 multicast DMX data over WiFi
2. Outputs physical DMX512 via RS485
3. Supports offline playback (loop buffer or stored chase)
4. Displays status on OLED

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    AETHER Pulse Node                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  WiFi ──▶ sACN/E1.31 ──▶ DMX Buffer ──▶ esp_dmx ──▶ DMX Output  │
│           (239.255.0.x)   (dmxIn[513])              (RS485)      │
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │ ESPAsyncE131 │    │  Loop Buffer │    │  Chase Store │       │
│  │  (receiver)  │    │  (80 frames) │    │   (NVS)      │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
│         │                   │                    │               │
│         └───────────────────┴────────────────────┘               │
│                            │                                     │
│                   ┌────────▼────────┐                           │
│                   │ Offline Playback│                           │
│                   │   Controller    │                           │
│                   └─────────────────┘                           │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Pin Configuration

| Pin | Function | Description |
|-----|----------|-------------|
| GPIO 17 | DMX_TX | DMX data output |
| GPIO 16 | DMX_RX | DMX data input (unused) |
| GPIO 4 | DMX_ENABLE | RS485 direction control |
| GPIO 21 | SDA | OLED I2C data |
| GPIO 22 | SCL | OLED I2C clock |

---

## Network Configuration

| Parameter | Value |
|-----------|-------|
| SSID | `AetherDMX` |
| Password | `lettherebelight` |
| Controller IP | `192.168.50.1` |
| Config UDP Port | 5555 |
| Discovery UDP Port | 5556 |
| sACN Port | 5568 |

---

## Timing Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| DMX_OUTPUT_FPS | 40 | Fixed DMX output rate |
| SACN_TIMEOUT_MS | 3000 | Switch to offline after 3s |
| STATUS_LOG_INTERVAL_MS | 10000 | Serial status every 10s |
| LOOP_BUFFER_FRAMES | 80 | ~2 seconds at 40fps |
| LOOP_BUFFER_CHANNELS | 128 | First 128 channels buffered |
| MAX_CHASE_STEPS | 16 | Max steps in stored chase |

---

## Offline Playback Modes

### OFFLINE_NONE (0)
- Gradual fade to black when sACN lost
- Default for safety (lights go off)

### OFFLINE_LOOP (1) - Default
- Records last ~2 seconds of DMX input
- Loops the recorded frames when offline
- Seamless transition back when online

### OFFLINE_HOLD (3)
- Holds last received frame indefinitely
- Simple but may cause stuck states

### OFFLINE_CHASE (2)
- Plays pre-stored chase definition from NVS
- Chase stored via UDP command from Pi
- Includes fade times between steps

---

## UDP Commands (Port 5555)

### Configure Node

```json
{
  "cmd": "configure",
  "universe": 2,
  "slice_start": 1,
  "slice_end": 128,
  "name": "Kitchen",
  "pair": true
}
```

**Fields:**
- `universe` - sACN universe to listen on (1-63999)
- `slice_start` - First DMX channel to output (1-512)
- `slice_end` - Last DMX channel to output (1-512)
- `name` - Human-readable node name
- `pair` - Mark node as paired

### Set Offline Mode

```json
{
  "cmd": "set_offline_mode",
  "mode": "loop"
}
```

**Modes:** `none`, `loop`, `chase`, `hold`

### Store Chase for Offline

```json
{
  "cmd": "store_chase",
  "loop_count": 0,
  "steps": [
    {
      "channels": [255, 128, 64, 0, ...],
      "fade_ms": 500,
      "hold_ms": 1000
    },
    {
      "channels": [0, 255, 0, 0, ...],
      "fade_ms": 500,
      "hold_ms": 1000
    }
  ]
}
```

**Fields:**
- `loop_count` - Number of loops (0 = infinite)
- `steps[].channels` - Array of 128 channel values
- `steps[].fade_ms` - Fade time to this step
- `steps[].hold_ms` - Hold time at this step

### Get Offline Status

```json
{"cmd": "offline_status"}
```

**Response (serial):**
```
Offline: mode=loop, buffer=80 frames, chase=3 steps, active=NO
```

### Clear Configuration

```json
{"cmd": "clear_config"}
```

Resets node to unpaired state, clears universe assignment.

---

## NVS Storage

Configuration stored in ESP32 Non-Volatile Storage (NVS):

### "aether" namespace:
| Key | Type | Description |
|-----|------|-------------|
| is_paired | bool | Node paired status |
| universe | int | sACN universe |
| slice_start | int | First channel |
| slice_end | int | Last channel |
| slice_mode | int | 0=zero_outside, 1=pass_through |
| name | string | Node name |
| offline_mode | int | Offline playback mode |

### "chase" namespace:
| Key | Type | Description |
|-----|------|-------------|
| step_count | uint8 | Number of chase steps |
| loop_count | uint8 | Loop count (0=infinite) |
| s0_fade | uint16 | Step 0 fade time |
| s0_hold | uint16 | Step 0 hold time |
| s0_ch | bytes | Step 0 channel values |
| s1_fade | ... | ... |

---

## Serial Output Format

### Status Report (every 10 seconds)

```
────────────────────────────────────────────────────
STATUS @ 312s | pulse-422C | Universe 2
────────────────────────────────────────────────────
  Slice:    1-512 (zero_outside)
  sACN:     LIVE (last 125ms ago)
  Sender:   6CF3..5A3D (changes: 0)
  Packets:  RX=1047, ignored=0
  Output:   TX=11951, fps=38.4 (target 40)
  WiFi:     OK, RSSI=-52 dBm
  Preview:  ch1-4=[255,128,64,200]
  Boundary: [255]@0, [128]@1 | [0]@512, [0]@513
────────────────────────────────────────────────────
```

### Offline Mode Messages

```
═══════════════════════════════════════════════════
  OFFLINE MODE: LOOP
  Buffer: 80 frames (~2.0s)
═══════════════════════════════════════════════════
ONLINE: Back after 394ms offline
```

---

## OLED Display

128x64 pixel display showing:
- **Line 1:** Node name or ID
- **Line 2:** Universe assignment
- **Line 3:** Channel slice (e.g., "Ch 1-128")
- **Line 4:** sACN status (LIVE, TIMEOUT, OFFLINE mode)
- **Line 5:** Stats (RX count, sender changes)

---

## Build & Flash

```bash
# Build
cd aether-pulse/hybrid
pio run -e pulse

# Flash via USB
pio run -e pulse -t upload

# Monitor serial
pio device monitor -b 115200
```

---

## OTA Updates

Firmware supports OTA (Over-The-Air) updates via ArduinoOTA.

```bash
# Flash via WiFi (from Pi)
pio run -e pulse -t upload --upload-port 192.168.50.123
```

---

## Troubleshooting

### Node shows RX=0
- Check WiFi connection (RSSI in status)
- Verify sACN multicast reaching network
- Check OLA configuration on Pi
- Ensure DMX refresh loop is running

### All channels zero
- Check if scene/chase is active on Pi
- Verify universe assignment matches
- Test with: `curl -X POST .../api/dmx/set -d '{"universe":2,"channels":{"1":255}}'`

### Fade to black messages
- sACN packets arriving but all zeros
- Or intermittent packet loss
- Check Pi's DMX state: `curl .../api/dmx/universe/2`

### Offline mode keeps triggering
- Packet loss or jitter on network
- Increase SACN_TIMEOUT_MS if needed
- Check WiFi signal strength (RSSI)

---

## Data Flow Diagram

```
Pi (aether-core)                    ESP32 (pulse node)
─────────────────                   ──────────────────

dmx_state.universes[2]
       │
       ▼
DMX Refresh Loop (40fps)
       │
       ▼
ola_set_dmx -u 2 -d ...
       │
       ▼
OLA (olad)
       │
       ▼
sACN Multicast ─────────────────▶  ESPAsyncE131.pull()
239.255.0.2:5568                           │
                                           ▼
                                    processSacnPacket()
                                           │
                                           ▼
                                    dmxIn[513] buffer
                                           │
                                    ┌──────┴──────┐
                                    ▼             ▼
                              recordFrame()   apply slice
                              (loop buffer)        │
                                                   ▼
                                           dmx_write(dmxIn)
                                                   │
                                                   ▼
                                            RS485 DMX Out
```
