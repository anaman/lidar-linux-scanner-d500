#!/usr/bin/env python3
"""
relay_lora.py — Forward lidar scans over a transparent UART radio link, e.g.
the Waveshare SX1262 LoRa HAT (which is just a wireless serial bridge).

The SX1262 HAT appears as another serial port (on a Raspberry Pi it is usually
/dev/ttyS0; via its on-board CP2102 USB it shows up as /dev/ttyUSBn). This
script reads the lidar, then writes framed packets into that port.

Frame format (written byte stream; receiver re-syncs on 0xAA 0x55):
    byte 0      : 0xAA
    byte 1      : 0x55
    byte 2      : mode  (0x00 = full scan, 0x01 = sector summary)
    byte 3..4   : count (uint16 LE)
    byte 5..    : payload
    last byte   : crc8 over all preceding bytes (ld_lidar.crc8)

    mode 0x00 "full"    payload = count * (angle u16 0.01deg | dist u16 mm | int u8)
    mode 0x01 "summary" payload = count * (min distance u16 mm per sector)

Modes:
    summary (default)  Lowest-bandwidth, practical over LoRa at 9600 baud.
                       Reduces each scan to the closest obstacle in each of
                       `--bins` angular sectors. Sector i covers angle
                       [i*360/bins, (i+1)*360/bins).
    full               Complete point cloud. Needs a high-speed link (see below).
    json               Human-readable, one JSON array per line (debugging).

Bandwidth reality: a full scan is ~500 points (~22.5 kbit/s at 5 bytes/point at
10 Hz). The LoRa HAT's default UART is 9600 baud, far too slow for `full` mode —
use `summary` there. `full`/`json` are for a wired or high-speed radio link.

Usage:
    python3 relay_lora.py --port /dev/ttyUSB1 --lora-port /dev/ttyS0
    python3 relay_lora.py --port /dev/ttyUSB1 --lora-port /dev/ttyUSB2 --mode full --hz 2
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time

import ld_lidar

MODE_FULL = 0x00
MODE_SUMMARY = 0x01


def frame_full(scan):
    payload = b"".join(
        struct.pack("<HHB",
                    int(round(p.angle * 100.0)) % 36000, p.distance, p.intensity)
        for p in scan
    )
    return _frame(MODE_FULL, len(scan), payload)


def frame_summary(scan, bins):
    # Closest obstacle per angular sector.
    sector = 360.0 / bins
    minima = [0xFFFF] * bins
    for p in scan:
        idx = min(int(p.angle / sector), bins - 1)
        if p.distance < minima[idx]:
            minima[idx] = p.distance
    payload = struct.pack(f"<{bins}H", *minima)
    return _frame(MODE_SUMMARY, bins, payload)


def _frame(mode, count, payload):
    body = bytes((0xAA, 0x55, mode)) + struct.pack("<H", count) + payload
    return body + bytes((ld_lidar.crc8(body),))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="auto",
                    help="lidar serial port, or 'auto' to detect the CP210x device")
    ap.add_argument("--baud", type=int, default=230400, help="lidar baud rate")
    ap.add_argument("--lora-port", required=True,
                    help="radio/relay serial port (e.g. /dev/ttyS0 or /dev/ttyUSB2)")
    ap.add_argument("--lora-baud", type=int, default=9600)
    ap.add_argument("--mode", default="summary", choices=["summary", "full", "json"])
    ap.add_argument("--bins", type=int, default=90,
                    help="angular sectors for summary mode")
    ap.add_argument("--hz", type=float, default=0.0,
                    help="max scans per second (0 = unlimited)")
    args = ap.parse_args()

    import serial

    lidar = ld_lidar.LidarScanner(args.port, args.baud)
    lidar.open()
    radio = serial.Serial(args.lora_port, args.lora_baud, timeout=0.5)
    period = 0.0 if args.hz <= 0 else 1.0 / args.hz
    last = 0.0

    print(f"relaying lidar ({args.port}) -> radio ({args.lora_port}) "
          f"mode={args.mode}")
    try:
        for scan in lidar.iter_scans():
            if scan is None:
                continue
            if period and time.time() - last < period:
                continue
            last = time.time()
            if args.mode == "full":
                data = frame_full(scan)
            elif args.mode == "summary":
                data = frame_summary(scan, args.bins)
            else:  # json
                arr = [{"a": round(p.angle, 2), "d": p.distance,
                        "i": p.intensity} for p in scan]
                data = (json.dumps(arr) + "\n").encode()
            radio.write(data)
            print(f"tx {len(data)} bytes @ {time.strftime('%H:%M:%S')}")
    except KeyboardInterrupt:
        pass
    finally:
        radio.close()
        lidar.close()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as e:
        sys.exit(f"error: {e}")
