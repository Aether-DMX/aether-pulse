# AETHER Pulse sACN/E1.31 Runbook

## Architecture Overview

```
Pi (aether-core v4.0)
    |
    +-- OLA (Open Lighting Architecture)
            |
            +-- sACN/E1.31 multicast (239.255.0.{universe})
                    |
                    +-- ESP32 Nodes (pulse-sacn firmware v2.2.0)
                            |
                            +-- Channel Slice (configurable window)
                            |
                            +-- RS485/DMX output @ 40Hz (fixed rate)
```

**Transport**: sACN/E1.31 multicast over WiFi
**Config Commands**: UDP JSON on port 8888
**Discovery**: UDP broadcast on port 9999
**Multi-sender policy**: last-wins (documented below)

---

## Key Design Principles

### 1. Multi-Universe Support (Universe > 1)
- Universe is a first-class parameter, never hardcoded to 1
- Pi-side `dmx_state.universes` uses universe as dictionary key
- ESP32 nodes are configurable for any universe (1-63999)
- No flattening of (universe, channel) to just channel

### 2. Receive/Output Timing Separation
- **ESP32**: sACN reception updates buffer, DMX output runs at fixed 40Hz
- **Pi**: On-demand output via OLA (controller-driven)
- Receive never directly triggers output - decoupled via buffer

### 3. Multi-Sender Policy: Last-Wins
- If multiple sACN sources transmit to same universe, last packet wins
- Sender CID tracked per universe
- Sender changes logged (rate-limited to 1 per 5 seconds)
- No priority-based selection (simple deterministic behavior)

---

## Installation Steps

### 1. Pi Side (aether-core)

```bash
# Install OLA
sudo apt-get update
sudo apt-get install ola ola-python

# Configure OLA for sACN output
# Edit /etc/ola/ola-e131.conf:
#   ip = ""  (bind to all interfaces)
#   use_priority = true
sudo systemctl enable olad
sudo systemctl start olad

# Verify OLA is running
ola_dev_info

# Start aether-core
cd /path/to/aether-core
python3 aether-core.py
```

**Expected startup output:**
```
============================================================
AETHER DMX Controller v4.0.0
Transport: sACN/E1.31 via OLA
============================================================
OLA/sACN output enabled
API server on port 8891
Discovery on UDP 9999
OLA/sACN output enabled
============================================================
```

### 2. ESP32 Nodes (aether-pulse)

```bash
# Build firmware
cd aether-pulse/hybrid
pio run

# Flash to ESP32
pio run -t upload

# Monitor serial output
pio device monitor
```

**Expected boot output (v2.2.0):**
```
═══════════════════════════════════════════════════
  AETHER Pulse - sACN/E1.31 DMX Node
═══════════════════════════════════════════════════
  Firmware:  pulse-sacn v2.2.0
  Node ID:   pulse-XXXX
  Transport: sACN/E1.31 multicast
  Output:    40 fps (fixed rate)
  Policy:    multi-sender=last-wins
═══════════════════════════════════════════════════

Config: Universe=1, Slice=1-512 (zero_outside), Name=PULSE-XXXX
Connecting to AetherDMX...
WiFi connected! IP: 192.168.50.xxx
sACN: Listening on Universe 1 (multicast 239.255.0.1)
DMX: Output initialized @ 40 fps (UART1)
```

---

## Configuration

### ESP32 Node Settings (via NVS)

| Setting | Default | Description |
|---------|---------|-------------|
| `node_id` | `pulse-XXXX` | Auto-generated from MAC |
| `node_name` | `PULSE-XXXX` | Display name |
| `universe` | `1` | sACN universe (1-63999) |
| `channel_start` | `1` | First DMX channel (slice start) |
| `channel_end` | `512` | Last DMX channel (slice end) |
| `slice_mode` | `zero_outside` | `zero_outside` or `pass_through` |

### Channel Slice Feature (v2.2.0)

Multiple nodes can share a single universe, each outputting a different channel window:
- **Node A**: Universe 1, channels 1-256 (slice_mode=zero_outside)
- **Node B**: Universe 1, channels 257-512 (slice_mode=zero_outside)

Each node outputs a full 512-channel DMX frame:
- **zero_outside** (default): Channels outside slice forced to 0
- **pass_through**: All 512 channels pass through unchanged

### Change Universe via UDP JSON

```bash
# Send config command to node
echo '{"cmd":"config","universe":2}' | nc -u <node_ip> 8888

# Configure slice (channels 1-256, zero outside)
echo '{"cmd":"config","channel_start":1,"channel_end":256,"slice_mode":"zero_outside"}' | nc -u <node_ip> 8888

# Configure slice (channels 257-512)
echo '{"cmd":"config","channel_start":257,"channel_end":512}' | nc -u <node_ip> 8888

# Request status
echo '{"cmd":"status"}' | nc -u <node_ip> 8888
```

### WiFi Network

| Setting | Value |
|---------|-------|
| SSID | `AetherDMX` |
| Password | (none - open network) |
| Pi IP | `192.168.50.1` |
| Node IP range | `192.168.50.100-199` |

---

## Verification Checklist

### Pi Side Checks

- [ ] OLA daemon running: `systemctl status olad`
- [ ] OLA devices visible: `ola_dev_info`
- [ ] sACN plugin enabled: `ola_plugin_info | grep E1.31`
- [ ] aether-core running without errors
- [ ] API responding: `curl http://localhost:8891/api/status`

### ESP32 Node Checks

- [ ] Serial shows `pulse-sacn v2.2.0`
- [ ] WiFi connected to `AetherDMX`
- [ ] sACN multicast joined (`Listening on Universe X`)
- [ ] DMX output at 40 fps
- [ ] OLED shows node status
- [ ] Status logs every 10 seconds

---

## Manual Test Procedures

### Test 1: Multi-Universe Verification

**Goal**: Confirm universes > 1 work correctly end-to-end.

```bash
# Configure node for universe 3
echo '{"cmd":"config","universe":3}' | nc -u <node_ip> 8888

# Send DMX to universe 3 via OLA
ola_set_dmx -u 3 -d 255,128,64,32,0,0

# Expected ESP32 serial output:
# sACN: Listening on Universe 3 (multicast 239.255.0.3)
# STATUS @ Xs | pulse-XXXX | Universe 3
#   sACN:     LIVE (last 0ms ago)
#   Channels: 1-512, preview=[255,128,64,32]
```

**Pass criteria**:
- ESP32 receives data on universe 3
- DMX output shows channels 1-4 with values 255,128,64,32
- No universe-1 fallback behavior

### Test 2: Packet Rate Stress Test

**Goal**: Confirm output rate stays stable under high input rate.

```bash
# Send packets rapidly (60+ fps) from Pi
while true; do
  ola_set_dmx -u 1 -d $((RANDOM % 256)),$((RANDOM % 256)),$((RANDOM % 256))
  sleep 0.01
done
```

**Monitor ESP32 status log:**
```
STATUS @ 30s | pulse-XXXX | Universe 1
  sACN:     LIVE (last 5ms ago)
  Packets:  RX=1847, dropped=0
  Output:   TX=1200, fps=40.0 (target 40)
```

**Pass criteria**:
- Output fps remains ~40 (not 60+)
- No dropped packets
- CPU stable (no watchdog resets)

### Test 3: Multi-Sender Collision Test

**Goal**: Confirm last-wins policy works and is logged.

```bash
# Terminal 1: Send from OLA (source A)
ola_set_dmx -u 1 -d 100,100,100,100

# Terminal 2: Send from QLC+ or another sACN source (source B)
# Configure QLC+ to output sACN on universe 1
```

**Expected ESP32 serial output:**
```
sACN: First sender detected for Universe 1
⚠️ Universe 1 sender changed: A1B2C3D4 -> E5F6G7H8 (policy=last-wins)
```

**Pass criteria**:
- No crash when senders switch
- Sender change logged (rate-limited)
- Last sender's data is output
- OLED shows sender changes count (`S:X`)

### Test 4: Channel Slice - Shared Universe

**Goal**: Confirm two nodes can share a universe with different channel slices.

```bash
# Configure Node A for channels 1-256
echo '{"cmd":"config","universe":1,"channel_start":1,"channel_end":256,"slice_mode":"zero_outside"}' | nc -u <node_a_ip> 8888

# Configure Node B for channels 257-512
echo '{"cmd":"config","universe":1,"channel_start":257,"channel_end":512,"slice_mode":"zero_outside"}' | nc -u <node_b_ip> 8888

# Send full-universe test pattern
ola_set_dmx -u 1 -d $(python3 -c "print(','.join(str(i%256) for i in range(512)))")

# Expected Node A serial output:
# Config: Universe=1, Slice=1-256 (zero_outside), Name=PULSE-XXXX
# STATUS @ Xs | pulse-XXXX | Universe 1
#   Slice:    1-256 (zero_outside)
#   Channels: preview=[0,1,2,3] | boundary=ch256:255

# Expected Node B serial output:
# Config: Universe=1, Slice=257-512 (zero_outside), Name=PULSE-YYYY
# STATUS @ Xs | pulse-YYYY | Universe 1
#   Slice:    257-512 (zero_outside)
#   Channels: preview=[0,1,2,3] | boundary=ch257:0
```

**Pass criteria**:
- Node A outputs channels 1-256, channels 257-512 are 0
- Node B outputs channels 257-512, channels 1-256 are 0
- No interference between nodes
- Status shows slice configuration

### Test 5: Channel Slice - Boundary Test

**Goal**: Verify boundary channel values in slice output.

```bash
# Configure node for middle slice
echo '{"cmd":"config","channel_start":100,"channel_end":200}' | nc -u <node_ip> 8888

# Send test pattern: channel N = N (value equals channel number)
ola_set_dmx -u 1 -d $(python3 -c "print(','.join(str(i) for i in range(1,256)))")

# Expected serial status:
# STATUS @ Xs | pulse-XXXX | Universe 1
#   Slice:    100-200 (zero_outside)
#   Channels: preview=[100,101,102,103] | boundary=ch100:100,ch200:200
```

**Pass criteria**:
- Channels 1-99 output as 0
- Channels 100-200 output correctly (value = channel number)
- Channels 201-512 output as 0
- Boundary values logged correctly

### Test 6: Slice Mode - pass_through

**Goal**: Verify pass_through mode outputs all channels unchanged.

```bash
# Configure node for pass_through mode
echo '{"cmd":"config","channel_start":1,"channel_end":256,"slice_mode":"pass_through"}' | nc -u <node_ip> 8888

# Send test pattern
ola_set_dmx -u 1 -d $(python3 -c "print(','.join(str(i%256) for i in range(512)))")

# Expected: ALL 512 channels pass through unchanged (slice_start/end ignored)
```

**Pass criteria**:
- All 512 channels output unchanged
- Slice boundaries are not applied
- Status shows `slice_mode=pass_through`

### Test 7: Slice with Multi-Sender

**Goal**: Verify slice works correctly with sender switching.

```bash
# Configure node for slice
echo '{"cmd":"config","channel_start":1,"channel_end":256}' | nc -u <node_ip> 8888

# Send from two sources alternately
# (Use OLA and another sACN source like QLC+)

# Expected: Slice is applied to whichever sender is active (last-wins)
```

**Pass criteria**:
- Slice applied regardless of sender
- Sender changes logged normally
- No slice corruption on sender switch

### Test 8: Frontend Slice Configuration

**Goal**: Verify frontend can configure slices via API.

```bash
# Pair node with slice configuration via API
curl -X POST http://localhost:8891/api/nodes/<node_id>/pair \
  -H "Content-Type: application/json" \
  -d '{"name":"Test Node","universe":1,"channel_start":1,"channel_end":256}'

# Configure existing node with slice_mode
curl -X POST http://localhost:8891/api/nodes/<node_id>/configure \
  -H "Content-Type: application/json" \
  -d '{"channel_start":257,"channel_end":512,"slice_mode":"zero_outside"}'

# Verify node received config
echo '{"cmd":"status"}' | nc -u <node_ip> 8888
```

**Pass criteria**:
- API accepts channel_start, channel_end, slice_mode
- Node receives and applies configuration
- Frontend UI continues to work (backward compatible)

---

## Expected Log Examples

### Normal Operation (Success)
```
───────────────────────────────────────────────
STATUS @ 120s | pulse-A1B2 | Universe 1
───────────────────────────────────────────────
  sACN:     LIVE (last 12ms ago)
  Sender:   F0E1..D2C3 (changes: 0)
  Packets:  RX=4532, dropped=0
  Output:   TX=4800, fps=40.0 (target 40)
  WiFi:     OK, RSSI=-52 dBm
  Channels: 1-512, preview=[128,64,32,0]
───────────────────────────────────────────────
```

### Sender Collision (Two Sources)
```
sACN: First sender detected for Universe 1
───────────────────────────────────────────────
STATUS @ 60s | pulse-A1B2 | Universe 1
───────────────────────────────────────────────
  sACN:     LIVE (last 8ms ago)
  Sender:   A1B2..C3D4 (changes: 0)
  Packets:  RX=2400, dropped=0
  Output:   TX=2400, fps=40.0 (target 40)
  WiFi:     OK, RSSI=-58 dBm
  Channels: 1-512, preview=[255,0,0,0]
───────────────────────────────────────────────
⚠️ Universe 1 sender changed: A1B2C3D4 -> E5F6G7H8 (policy=last-wins)
───────────────────────────────────────────────
STATUS @ 70s | pulse-A1B2 | Universe 1
───────────────────────────────────────────────
  sACN:     LIVE (last 3ms ago)
  Sender:   E5F6..G7H8 (changes: 1)
  Packets:  RX=2800, dropped=0
  Output:   TX=2800, fps=40.0 (target 40)
  WiFi:     OK, RSSI=-58 dBm
  Channels: 1-512, preview=[0,255,0,0]
───────────────────────────────────────────────
```

### Signal Timeout (Fade to Black)
```
───────────────────────────────────────────────
STATUS @ 180s | pulse-A1B2 | Universe 1
───────────────────────────────────────────────
  sACN:     TIMEOUT (last 6234ms ago)
  Sender:   F0E1..D2C3 (changes: 0)
  Packets:  RX=7200, dropped=0
  Output:   TX=7200, fps=40.0 (target 40)
  WiFi:     OK, RSSI=-55 dBm
  Channels: 1-512, preview=[45,22,11,0]
───────────────────────────────────────────────
sACN: Fade to black complete
```

---

## Troubleshooting

### Node not receiving sACN

1. Check WiFi connected: Serial shows `WiFi connected!`
2. Check multicast joined: `sACN: Listening on Universe X`
3. Verify universe matches: Pi sending to same universe ESP32 is listening
4. Check firewall: sACN uses UDP port 5568
5. Check status log: `sACN: NONE` means no packets ever received

### DMX output not working

1. Check RS485 wiring: TX=17, RX=16, EN=4
2. Verify DMX cable polarity
3. Check serial output for `DMX: Output initialized`
4. Check status log: `Output: TX=X, fps=40.0`

### Sender collision issues

1. Check status log for `sender changed` messages
2. Sender change is logged with CID truncated (first/last 4 hex chars)
3. Policy is always last-wins - no priority support
4. If switching is too fast (< 5s), only first change is logged per interval

### OLA not outputting

```bash
# Check OLA E1.31 plugin
ola_plugin_info -p 4  # E1.31 plugin ID

# Manually send test
ola_streaming_client -u 1

# Check network interface
ola_e131 --help
```

---

## Files Reference

| File | Purpose |
|------|---------|
| `aether-core/aether-core.py` | Pi controller (v4.0.0) |
| `aether-pulse/hybrid/src/main.cpp` | ESP32 firmware (v2.2.0) |
| `aether-pulse/hybrid/platformio.ini` | Build config |
| `aether-pulse/hybrid/include/firmware_identity.h` | Version/identity |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 2.2.0 | 2024-12-30 | Channel slice feature: shared universe, configurable channel windows |
| 2.1.0 | 2024-12-30 | Multi-sender tracking (last-wins), status logging, timing separation |
| 2.0.0 | 2024-12-30 | Complete rewrite: sACN/E1.31 only, removed ESP-NOW/UART |

---

## Quick Commands

```bash
# Build firmware
cd aether-pulse/hybrid && pio run

# Flash ESP32
pio run -t upload

# Serial monitor
pio device monitor

# Test OLA output (universe 1)
ola_set_dmx -u 1 -d 255,128,64,32

# Test OLA output (universe 3)
ola_set_dmx -u 3 -d 255,128,64,32

# Configure node universe
echo '{"cmd":"config","universe":3}' | nc -u <node_ip> 8888

# Request node status
echo '{"cmd":"status"}' | nc -u <node_ip> 8888

# Check nodes via API
curl http://localhost:8891/api/nodes

# API diagnostics
curl http://localhost:8891/api/diagnostics
```

---

## Timing Reference

| Parameter | Value | Notes |
|-----------|-------|-------|
| DMX output rate | 40 fps (25ms) | Fixed, decoupled from receive |
| sACN timeout | 5000ms | After this, fade to black starts |
| Fade step interval | 50ms | Linear fade, -5 per step |
| Status log interval | 10000ms | Serial status every 10s |
| Heartbeat interval | 10000ms | UDP heartbeat to Pi |
| Sender change log rate limit | 5000ms | Max 1 log per 5s |
