#!/usr/bin/env python3
"""
Quick diagnostic: reads raw serial from ESP32 and shows exactly what
is being received + whether it parses correctly.

Usage: python3 serial_debug.py [port]
  Default port: /dev/ttyUSB0
"""
import serial
import sys
import time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
baud = 115200

print(f"Opening {port} at {baud} baud...")
try:
    ser = serial.Serial(port, baud, timeout=2.0)
except Exception as e:
    print(f"FAILED to open port: {e}")
    print("Make sure serial_imu_bridge is STOPPED (Ctrl+C) first!")
    sys.exit(1)

print(f"Connected! Reading 20 lines...\n")
print(f"{'LINE#':<6} {'RAW DATA':<70} {'PARSE STATUS'}")
print("=" * 120)

for i in range(20):
    try:
        raw = ser.readline()
        if not raw:
            print(f"{i+1:<6} {'<TIMEOUT - no data received>':<70} FAIL")
            continue
        
        # Show raw bytes
        line = raw.decode('utf-8', errors='replace').strip()
        
        # Try to parse like the bridge does
        parts = line.split(',')
        if len(parts) == 8:
            try:
                values = list(map(float, parts))
                q1_w, q1_x, q1_y, q1_z = values[0:4]
                q2_w, q2_x, q2_y, q2_z = values[4:8]
                status = f"OK  q1=({q1_w:.3f},{q1_x:.3f},{q1_y:.3f},{q1_z:.3f}) q2=({q2_w:.3f},{q2_x:.3f},{q2_y:.3f},{q2_z:.3f})"
            except ValueError as e:
                status = f"PARSE ERROR: {e}"
        else:
            status = f"WRONG FIELD COUNT: got {len(parts)}, expected 8"
        
        # Truncate raw for display
        display_raw = line[:67] + "..." if len(line) > 70 else line
        print(f"{i+1:<6} {display_raw:<70} {status}")
        
    except Exception as e:
        print(f"{i+1:<6} {'ERROR':<70} {e}")

ser.close()
print("\n" + "=" * 120)
print("Done. Close this and restart the serial bridge.")
