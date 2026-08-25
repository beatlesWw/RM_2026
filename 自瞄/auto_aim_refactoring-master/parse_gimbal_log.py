#!/usr/bin/env python3
"""Parse gimbal communication log and convert hex to readable values."""

import struct
import re

# Log data
log_data = """
[2026-02-07 17:04:12.448] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 22 AD 24 BF 00 00 00 00 00 00 00 00 C8 29 E1 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.453] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 58 5E 72 3F D3 5E 06 3D AF 80 40 BD 68 36 A2 3E 4F DC 24 3F EE 52 1E 3D 84 41 E1 BD 02 42 06 BC 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.458] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 4F DC 24 BF 00 00 00 00 00 00 00 00 84 41 E1 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.468] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 59 60 72 3F 50 64 06 3D 06 78 40 BD 7E 2A A2 3E 15 DB 24 3F 05 BA 20 BD 7D 48 E1 BD 05 5D 96 3B 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.469] [debug] [Gimbal] tx frame (29 bytes): 5A A5 02 E0 57 9E BE F5 A2 D4 30 E6 46 ED B1 AC 71 13 BE A4 28 86 AF 00 34 20 32 7F FE 
[2026-02-07 17:04:12.479] [debug] [Gimbal] tx frame (29 bytes): 5A A5 02 E0 57 9E BE 11 6C 84 2C C3 DA FD B0 AC 71 13 BE 66 86 24 AE 00 D3 54 31 7F FE 
[2026-02-07 17:04:12.480] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 E6 6B 72 3F B2 51 06 3D 72 50 40 BD 57 E6 A1 3E 6A 9C 24 3F 4D B2 DE BD 2A 17 E1 BD 1B 45 B9 3B 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.490] [debug] [Gimbal] tx frame (29 bytes): 5A A5 02 8A DA 9D BE C1 A2 06 AD 12 77 91 AF 55 8A 13 BE 7B 14 DA 2B 00 24 3C 30 7F FE 
[2026-02-07 17:04:12.495] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 9C 8D 72 3F EB 24 06 3D 31 B5 3F BD 58 1F A1 3E 01 E1 23 3F 26 E5 8B BE 01 77 E0 BD C0 4D EE 3C 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.500] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 01 E1 23 BF 00 00 00 00 00 00 00 00 01 77 E0 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.510] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 01 E1 23 BF 00 00 00 00 00 00 00 00 01 77 E0 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.520] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 01 E1 23 BF 00 00 00 00 00 00 00 00 01 77 E0 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.521] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 05 19 73 3F FF 76 04 3D 4C 10 3C BD 62 E5 9D 3E 95 AB 20 3F 2A 54 2B BF 1C 5F DC BD C6 D6 C2 3D 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.530] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 95 AB 20 BF 00 00 00 00 00 00 00 00 1C 5F DC 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.541] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 95 AB 20 BF 00 00 00 00 00 00 00 00 1C 5F DC 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.551] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 95 AB 20 BF 00 00 00 00 00 00 00 00 1C 5F DC 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.561] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 95 AB 20 BF 00 00 00 00 00 00 00 00 1C 5F DC 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.571] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 95 AB 20 BF 00 00 00 00 00 00 00 00 1C 5F DC 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.576] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 B8 91 75 3F C3 D4 FD 3C 60 2C 2D BD 8E 26 8E 3E 43 AB 10 3F EC DF D7 BF 10 E2 C9 BD BA 72 22 3C 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.581] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 43 AB 10 BF 00 00 00 00 00 00 00 00 10 E2 C9 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.591] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 99 86 76 3F 0D 55 FB 3C BB F6 2E BD 1F 59 87 3E 5A B1 09 3F CF C1 F3 BF D1 D9 C9 BD 3A F3 D2 BD 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.591] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 5A B1 09 BF 00 00 00 00 00 00 00 00 D1 D9 C9 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.602] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 5A B1 09 BF 00 00 00 00 00 00 00 00 D1 D9 C9 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.603] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 47 4F 77 3F F1 A8 F9 3C 77 17 32 BD 87 72 81 3E FD 9E 03 3F 42 C0 00 C0 64 8D CB BD 04 A7 27 BE 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.612] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 FD 9E 03 BF 00 00 00 00 00 00 00 00 64 8D CB 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.622] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 FD 9E 03 BF 00 00 00 00 00 00 00 00 64 8D CB 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.632] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 FD 9E 03 BF 00 00 00 00 00 00 00 00 64 8D CB 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.643] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 FD 9E 03 BF 00 00 00 00 00 00 00 00 64 8D CB 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.653] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 FD 9E 03 BF 00 00 00 00 00 00 00 00 64 8D CB 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.657] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 C7 4E 7A 3F 1E B6 F4 3C 6D 4A 4D BD 7C 42 4E 3E 63 D7 D0 3E F7 9E EA BF A8 C6 E0 BD 79 00 AA BE 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.663] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 63 D7 D0 BE 00 00 00 00 00 00 00 00 A8 C6 E0 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.673] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 63 D7 D0 BE 00 00 00 00 00 00 00 00 A8 C6 E0 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.683] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 63 D7 D0 BE 00 00 00 00 00 00 00 00 A8 C6 E0 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.686] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 BF 84 7B 3F 0F 09 F1 3C FF DD 5D BD 9E 04 34 3E 8E 6D B8 3E 52 54 D6 BF 43 91 ED BD A6 8C 8C BE 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.693] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 8E 6D B8 BE 00 00 00 00 00 00 00 00 43 91 ED 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.698] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 E5 E6 7B 3F AF D8 EE 3C 06 99 63 BD A4 CD 2A 3E 1C 73 AC 3E 77 8F D3 BF FD 9E F3 BD 25 5E 8A BE 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.704] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 1C 73 AC BE 00 00 00 00 00 00 00 00 FD 9E F3 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.714] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 A8 63 7C 3F 32 A1 EB 3C EA 97 6A BD 78 50 1E 3E 83 C4 9F 3E 6D 50 D3 BF CF 6E F9 BD F5 19 75 BE 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.714] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 83 C4 9F BE 00 00 00 00 00 00 00 00 CF 6E F9 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.724] [debug] [Gimbal] tx frame (29 bytes): 5A A5 01 83 C4 9F BE 00 00 00 00 00 00 00 00 CF 6E F9 3D 00 00 00 00 00 00 00 00 7F FE 
[2026-02-07 17:04:12.726] [debug] [Gimbal] rx frame (43 bytes): 5A A5 01 C2 C4 7C 3F A4 4C E8 3C 93 F0 6E BD C6 F6 13 3E A4 50 95 3E 39 AC DB BF 74 EE FC BD A6 8C 8C BE 00 00 00 00 00 00 7F FE 
"""

def hex_to_float(hex_bytes):
    """Convert 4 hex bytes (little-endian) to float."""
    return struct.unpack('<f', bytes(hex_bytes))[0]

def parse_rx_frame(hex_str):
    """Parse rx frame (43 bytes) - GimbalToVision."""
    hex_list = [int(x, 16) for x in hex_str.split()]
    if len(hex_list) != 43:
        return None
    
    # Structure: head[2], mode, q[4*4], yaw, yaw_vel, pitch, pitch_vel, bullet_speed, bullet_count[2], tail[2]
    mode = hex_list[2]
    q_w = hex_to_float(hex_list[3:7])
    q_x = hex_to_float(hex_list[7:11])
    q_y = hex_to_float(hex_list[11:15])
    q_z = hex_to_float(hex_list[15:19])
    yaw = hex_to_float(hex_list[19:23])
    yaw_vel = hex_to_float(hex_list[23:27])
    pitch = hex_to_float(hex_list[27:31])
    pitch_vel = hex_to_float(hex_list[31:35])
    bullet_speed = hex_to_float(hex_list[35:39])
    bullet_count = hex_list[39] | (hex_list[40] << 8)
    
    return {
        'mode': mode,
        'yaw': yaw,
        'yaw_vel': yaw_vel,
        'pitch': pitch,
        'pitch_vel': pitch_vel,
        'bullet_speed': bullet_speed,
        'bullet_count': bullet_count
    }

def parse_tx_frame(hex_str):
    """Parse tx frame (29 bytes) - VisionToGimbal."""
    hex_list = [int(x, 16) for x in hex_str.split()]
    if len(hex_list) != 29:
        return None
    
    # Structure: head[2], mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, tail[2]
    mode = hex_list[2]
    yaw = hex_to_float(hex_list[3:7])
    yaw_vel = hex_to_float(hex_list[7:11])
    yaw_acc = hex_to_float(hex_list[11:15])
    pitch = hex_to_float(hex_list[15:19])
    pitch_vel = hex_to_float(hex_list[19:23])
    pitch_acc = hex_to_float(hex_list[23:27])
    
    return {
        'mode': mode,
        'yaw': yaw,
        'yaw_vel': yaw_vel,
        'yaw_acc': yaw_acc,
        'pitch': pitch,
        'pitch_vel': pitch_vel,
        'pitch_acc': pitch_acc
    }

def format_sign(val):
    """Format float with sign."""
    if val >= 0:
        return f"+{val:.6f}"
    else:
        return f"{val:.6f}"

# Parse log
rx_pattern = re.compile(r'\[([^\]]+)\].*rx frame \(43 bytes\): (.+)')
tx_pattern = re.compile(r'\[([^\]]+)\].*tx frame \(29 bytes\): (.+)')

rx_frames = []
tx_frames = []

for line in log_data.strip().split('\n'):
    rx_match = rx_pattern.search(line)
    tx_match = tx_pattern.search(line)
    
    if rx_match:
        timestamp = rx_match.group(1)
        hex_data = rx_match.group(2).strip()
        parsed = parse_rx_frame(hex_data)
        if parsed:
            parsed['timestamp'] = timestamp
            rx_frames.append(parsed)
    elif tx_match:
        timestamp = tx_match.group(1)
        hex_data = tx_match.group(2).strip()
        parsed = parse_tx_frame(hex_data)
        if parsed:
            parsed['timestamp'] = timestamp
            tx_frames.append(parsed)

import math

# Print combined table with degrees
print("\n" + "=" * 100)
print("【下位机 -> 上位机】RX (云台状态)")
print("=" * 100)
print(f"{'时间(ms)':<12} | {'yaw°':>10} | {'yaw_vel°/s':>12} | {'pitch°':>10} | {'pitch_vel°/s':>12} | {'弹速':>6}")
print("-" * 100)
for f in rx_frames:
    ts = f['timestamp'].split()[-1]  # 只取时间
    yaw_deg = math.degrees(f['yaw'])
    yaw_vel_deg = math.degrees(f['yaw_vel'])
    pitch_deg = math.degrees(f['pitch'])
    pitch_vel_deg = math.degrees(f['pitch_vel'])
    print(f"{ts:<12} | {yaw_deg:>+10.3f} | {yaw_vel_deg:>+12.3f} | {pitch_deg:>+10.3f} | {pitch_vel_deg:>+12.3f} | {f['bullet_speed']:>6.1f}")

print("\n" + "=" * 110)
print("【上位机 -> 下位机】TX (控制指令, Δyaw/Δpitch为相对增量)")
print("=" * 110)
print(f"{'时间(ms)':<12} | {'模式':>6} | {'Δyaw°':>10} | {'yaw_vel':>10} | {'yaw_acc':>10} | {'Δpitch°':>10} | {'pitch_vel':>10} | {'pitch_acc':>10}")
print("-" * 110)
for f in tx_frames:
    ts = f['timestamp'].split()[-1]
    mode_str = {0: '无目标', 1: '控制', 2: '开火'}.get(f['mode'], str(f['mode']))
    yaw_deg = math.degrees(f['yaw'])
    pitch_deg = math.degrees(f['pitch'])
    yaw_vel_deg = math.degrees(f['yaw_vel'])
    pitch_vel_deg = math.degrees(f['pitch_vel'])
    print(f"{ts:<12} | {mode_str:>6} | {yaw_deg:>+10.3f} | {yaw_vel_deg:>+10.3f} | {f['yaw_acc']:>+10.5f} | {pitch_deg:>+10.3f} | {pitch_vel_deg:>+10.3f} | {f['pitch_acc']:>+10.5f}")
