#!/usr/bin/env python3
"""
AETHER ESP-NOW Gateway - Serial Test Sender

Sends test DMX frames over serial to the ESP-NOW Gateway.
Useful for testing without the full AETHER Portal.

Usage:
    python3 serial_sender.py /dev/ttyUSB0 --pattern sine
    python3 serial_sender.py /dev/ttyACM0 --universe 2 --fps 40
"""

import argparse
import math
import serial
import struct
import sys
import time
from typing import Optional

# ═══════════════════════════════════════════════════════════════════════════════
# CRC16-CCITT Implementation
# ═══════════════════════════════════════════════════════════════════════════════

CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
]


def crc16_ccitt(data: bytes) -> int:
    """Calculate CRC16-CCITT checksum."""
    crc = 0xFFFF
    for byte in data:
        index = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ CRC16_TABLE[index]) & 0xFFFF
    return crc


# ═══════════════════════════════════════════════════════════════════════════════
# Pattern Generators
# ═══════════════════════════════════════════════════════════════════════════════

def pattern_sine(frame: int) -> bytes:
    """Generate sine wave pattern on channels 1-16."""
    data = bytearray(512)
    phase = frame * 0.05
    for i in range(16):
        channel_phase = phase + (i * 0.4)
        value = int((math.sin(channel_phase) + 1.0) * 127.5)
        data[i] = value
    # Master dimmer on channel 512
    data[511] = int((math.sin(phase * 0.5) + 1.0) * 127.5)
    return bytes(data)


def pattern_chase(frame: int) -> bytes:
    """Generate chase pattern on channels 1-16."""
    data = bytearray(512)
    active = (frame // 4) % 16
    data[active] = 255
    data[(active + 15) % 16] = 64
    data[(active + 1) % 16] = 64
    data[511] = 128
    return bytes(data)


def pattern_full(frame: int) -> bytes:
    """All channels at full."""
    return bytes([255] * 512)


def pattern_ramp(frame: int) -> bytes:
    """Linear ramp across channels."""
    data = bytearray(512)
    offset = (frame * 2) % 256
    for i in range(256):
        data[i] = (i + offset) % 256
    for i in range(256, 512):
        data[i] = (511 - i + offset) % 256
    return bytes(data)


def pattern_rgb(frame: int) -> bytes:
    """RGB fade on first 3 channels."""
    data = bytearray(512)
    phase = frame * 0.03
    data[0] = int((math.sin(phase) + 1.0) * 127.5)  # Red
    data[1] = int((math.sin(phase + 2.094) + 1.0) * 127.5)  # Green
    data[2] = int((math.sin(phase + 4.189) + 1.0) * 127.5)  # Blue
    return bytes(data)


PATTERNS = {
    'sine': pattern_sine,
    'chase': pattern_chase,
    'full': pattern_full,
    'ramp': pattern_ramp,
    'rgb': pattern_rgb,
}


# ═══════════════════════════════════════════════════════════════════════════════
# Frame Builder
# ═══════════════════════════════════════════════════════════════════════════════

def build_frame(universe: int, seq: int, dmx_data: bytes) -> bytes:
    """
    Build a serial frame for the ESP-NOW gateway.
    
    Frame format:
        Preamble: 0xAA 0x55 (2 bytes)
        Universe: uint16 LE (2 bytes)
        Sequence: uint32 LE (4 bytes)
        Length: uint16 LE (2 bytes)
        Payload: DMX data (512 bytes)
        CRC16: uint16 LE (2 bytes)
    
    Total: 524 bytes
    """
    # Ensure DMX data is exactly 512 bytes
    dmx_data = bytes(dmx_data[:512]).ljust(512, b'\x00')
    
    # Build header (excluding preamble for CRC calculation)
    header = struct.pack('<HLH', universe, seq, 512)
    
    # Calculate CRC over header + payload
    crc_data = header + dmx_data
    crc = crc16_ccitt(crc_data)
    
    # Build complete frame
    frame = b'\xAA\x55' + crc_data + struct.pack('<H', crc)
    
    return frame


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='AETHER ESP-NOW Gateway Serial Test Sender'
    )
    parser.add_argument(
        'port',
        help='Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0, COM3)'
    )
    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=921600,
        help='Baud rate (default: 921600)'
    )
    parser.add_argument(
        '--universe', '-u',
        type=int,
        default=1,
        help='DMX universe (default: 1)'
    )
    parser.add_argument(
        '--fps', '-f',
        type=int,
        default=40,
        help='Frames per second (default: 40)'
    )
    parser.add_argument(
        '--pattern', '-p',
        choices=list(PATTERNS.keys()),
        default='sine',
        help='Test pattern (default: sine)'
    )
    parser.add_argument(
        '--count', '-c',
        type=int,
        default=0,
        help='Number of frames to send (0 = infinite)'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Verbose output'
    )
    
    args = parser.parse_args()
    
    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Opened {args.port} at {args.baud} baud")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    
    # Get pattern generator
    pattern_func = PATTERNS[args.pattern]
    
    # Calculate frame interval
    frame_interval = 1.0 / args.fps
    
    print(f"Sending {args.pattern} pattern to universe {args.universe} at {args.fps} FPS")
    print("Press Ctrl+C to stop")
    print()
    
    seq = 0
    frame_count = 0
    start_time = time.time()
    last_print_time = start_time
    
    try:
        while True:
            frame_start = time.time()
            
            # Generate DMX data
            dmx_data = pattern_func(seq)
            
            # Build and send frame
            frame = build_frame(args.universe, seq, dmx_data)
            ser.write(frame)
            
            seq += 1
            frame_count += 1
            
            # Verbose output
            if args.verbose and seq % 40 == 0:
                print(f"Sent: seq={seq} Ch1-8=[{dmx_data[0]},{dmx_data[1]},{dmx_data[2]},{dmx_data[3]},{dmx_data[4]},{dmx_data[5]},{dmx_data[6]},{dmx_data[7]}]")
            
            # Periodic stats
            current_time = time.time()
            if current_time - last_print_time >= 5.0:
                elapsed = current_time - start_time
                actual_fps = frame_count / elapsed
                print(f"Stats: frames={frame_count}, actual_fps={actual_fps:.1f}, seq={seq}")
                last_print_time = current_time
            
            # Check count limit
            if args.count > 0 and frame_count >= args.count:
                break
            
            # Wait for next frame
            elapsed = time.time() - frame_start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        ser.close()
        
    # Final stats
    elapsed = time.time() - start_time
    actual_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"\nSummary: {frame_count} frames in {elapsed:.1f}s ({actual_fps:.1f} FPS)")


if __name__ == '__main__':
    main()
