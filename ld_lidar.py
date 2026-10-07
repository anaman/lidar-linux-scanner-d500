#!/usr/bin/env python3
"""
ld_lidar.py — Pure-Python driver for the Waveshare D600 LiDAR scanner
(LDRobot LD06 / LD19 serial protocol; the same hardware that Waveshare's
`ld_desktop` Windows tool visualizes).

Verified against a D600 / LD06 unit at 230400 baud.

Only depends on `pyserial`. No numpy/C extensions are required for parsing.

--------------------------------------------------------------------------------
Wire protocol (little-endian, 47-byte packets, continuous stream)
--------------------------------------------------------------------------------
  offset  size  field
  0       1     header       = 0x54
  1       1     ver_len      = 0x2C
  2       2     speed        deg/s  (uint16 LE)
  4       2     start_angle  0.01 deg (uint16 LE)
  6       36    12 points    each = distance uint16 LE (mm) + intensity uint8
  42      2     end_angle    0.01 deg (uint16 LE)
  44      2     timestamp    ms (uint16 LE)
  46      1     crc8         over bytes 0..45

CRC-8 is MSB-first, polynomial 0x4D, initial value 0x00, no reflection and no
final XOR (full polynomial 0x14D). This is the exact table published in the
official LDRobot SDK (ldlidar_stl_sdk).

Point angle (degrees) for point i in 0..11:
    diff  = (end_angle + 36000 - start_angle) % 36000      # 0.01 deg units
    angle = start_angle/100.0 + i * (diff / 11.0) / 100.0  # 12 pts -> 11 gaps
    angle is wrapped to [0, 360)
--------------------------------------------------------------------------------
"""
from __future__ import annotations

import struct
import sys
import time

import serial

PKG_HEADER = 0x54
PKG_VER_LEN = 0x2C
POINT_PER_PACK = 12
PACKET_LEN = 47
CRC_POLY = 0x4D
DEFAULT_BAUD = 230400

# A scan wraps when the angle falls from ~>340deg back down to ~<20deg.
WRAP_HIGH = 340.0
WRAP_LOW = 20.0


def list_serial_ports():
    """Return a list of human-readable ``device (description)`` strings."""
    from serial.tools import list_ports

    return [f"{p.device}  ({p.description or 'unknown'})" for p in list_ports.comports()]


def find_lidar_port():
    """Auto-detect the lidar's serial port.

    The LD06/LD19 lidar enumerates as a Silicon Labs CP210x UART bridge
    (USB VID 0x10C4). Returns the device path (e.g. ``/dev/ttyUSB1``), or
    ``None`` if there is not exactly one match.
    """
    import glob
    import os
    from serial.tools import list_ports

    hits = []

    # 1) Prefer the stable /dev/serial/by-id symlink.
    for d in sorted(glob.glob("/dev/serial/by-id/*")):
        if "cp210" in os.path.basename(d).lower():
            hits.append(os.path.realpath(d))

    # 2) Match by USB vendor id (Silicon Labs) or description/hwid text.
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if p.vid == 0x10C4 or "cp210" in desc or "cp210" in hwid:
            if p.device not in hits:
                hits.append(p.device)

    # De-duplicate while preserving order.
    uniq = []
    for h in hits:
        if h not in uniq:
            uniq.append(h)

    return uniq[0] if len(uniq) == 1 else None


def crc8(data: bytes) -> int:
    """CRC-8, MSB-first, poly 0x4D, init 0x00 (LDRobot LD06/LD19)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class LidarPoint:
    """One range measurement (polar)."""
    __slots__ = ("angle", "distance", "intensity")

    def __init__(self, angle: float, distance: int, intensity: int):
        self.angle = angle          # degrees, [0, 360)
        self.distance = distance    # millimeters
        self.intensity = intensity  # 0..255

    def __repr__(self):
        return (
            f"LidarPoint(angle={self.angle:.2f}deg, "
            f"distance={self.distance}mm, intensity={self.intensity})"
        )


class LidarPacket:
    """One decoded 47-byte protocol packet."""
    __slots__ = ("speed", "start_angle", "end_angle", "timestamp", "points")

    def __init__(self, speed, start_angle, end_angle, timestamp, points):
        self.speed = speed              # deg/s
        self.start_angle = start_angle  # degrees
        self.end_angle = end_angle      # degrees
        self.timestamp = timestamp      # ms
        self.points = points            # list[LidarPoint] (length 12)

    def __repr__(self):
        return (
            f"LidarPacket(speed={self.speed}dps, "
            f"{self.start_angle:.2f}..{self.end_angle:.2f}deg, "
            f"{len(self.points)}pts)"
        )


def parse_packet(raw: bytes) -> LidarPacket | None:
    """Decode one 47-byte packet. Returns None on bad header/CRC."""
    if len(raw) != PACKET_LEN:
        return None
    if raw[0] != PKG_HEADER or raw[1] != PKG_VER_LEN:
        return None
    if crc8(raw[:46]) != raw[46]:
        return None

    speed, start = struct.unpack_from("<HH", raw, 2)
    end, ts = struct.unpack_from("<HH", raw, 42)
    # 12 points: distance u16 LE (mm) + intensity u8, starting at offset 6.
    diff = (end + 36000 - start) % 36000          # 0.01 deg units
    step = diff / float(POINT_PER_PACK - 1)       # 0.01 deg units per point
    points = []
    for i in range(POINT_PER_PACK):
        off = 6 + i * 3
        distance, intensity = struct.unpack_from("<HB", raw, off)
        angle = start / 100.0 + i * (step / 100.0)
        if angle >= 360.0:
            angle -= 360.0
        points.append(LidarPoint(angle, distance, intensity))

    return LidarPacket(speed, start / 100.0, end / 100.0, ts, points)


class LidarScanner:
    """Open the lidar serial port and read full 360-degree scans."""

    def __init__(self, port: str = "auto", baud: int = DEFAULT_BAUD, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: serial.Serial | None = None

        # Telemetry from the most recent packet.
        self.speed_dps = 0.0
        self.last_timestamp = 0
        self.packets_ok = 0
        self.packets_bad = 0

        # Scan assembly state.
        self._scan: list[LidarPoint] = []
        self._ready: list[LidarPoint] | None = None
        self._prev_angle: float | None = None
        self._first_wrap = False

    # -- lifecycle ----------------------------------------------------------
    def open(self) -> "LidarScanner":
        if self.port in (None, "", "auto"):
            port = find_lidar_port()
            if port is None:
                avail = list_serial_ports()
                raise RuntimeError(
                    "Could not auto-detect the lidar (no unique CP210x USB-serial "
                    "port found). Available ports: "
                    + (", ".join(avail) if avail else "none")
                    + ". Pass --port <device> explicitly."
                )
            self.port = port
            print(f"[ld_lidar] auto-detected lidar on {port}", file=sys.stderr)
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self.ser.reset_input_buffer()
        return self

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self) -> "LidarScanner":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def speed_hz(self) -> float:
        """Rotation speed in Hz (deg/s / 360)."""
        return self.speed_dps / 360.0

    # -- packet level -------------------------------------------------------
    def _read_packet(self, timeout: float) -> LidarPacket | None:
        """Read one valid packet, resynchronizing on the 0x54/0x2C header.

        `timeout` is applied to each low-level read; use 0 for a non-blocking
        poll and a small value (e.g. 0.05) to let a half-arrived packet finish.
        """
        ser = self.ser
        old = ser.timeout
        ser.timeout = timeout
        try:
            while True:
                h = ser.read(1)
                if not h:
                    return None
                if h[0] != PKG_HEADER:
                    continue
                v = ser.read(1)
                if not v:
                    return None
                if v[0] != PKG_VER_LEN:
                    continue
                rest = ser.read(PACKET_LEN - 2)
                if len(rest) != PACKET_LEN - 2:
                    return None
                raw = bytes((PKG_HEADER, PKG_VER_LEN)) + rest
                pkt = parse_packet(raw)
                if pkt is None:
                    self.packets_bad += 1
                    continue
                self.packets_ok += 1
                self.speed_dps = pkt.speed
                self.last_timestamp = pkt.timestamp
                return pkt
        finally:
            ser.timeout = old

    def read_packet(self, timeout: float | None = None) -> LidarPacket | None:
        """Block until one valid packet arrives (None on timeout)."""
        return self._read_packet(self.timeout if timeout is None else timeout)

    # -- scan level ---------------------------------------------------------
    def _push(self, points) -> None:
        """Feed packet points into the scan assembler."""
        for p in points:
            if (
                self._prev_angle is not None
                and self._prev_angle > WRAP_HIGH
                and p.angle < WRAP_LOW
            ):
                # Crossed the 360/0 seam -> a full revolution just completed.
                if self._first_wrap:
                    self._ready = self._scan
                self._first_wrap = True
                self._scan = []
            self._scan.append(p)
            self._prev_angle = p.angle

    def poll_scan(self) -> list[LidarPoint] | None:
        """Drain whatever has arrived and return a full scan if one completed."""
        while True:
            pkt = self._read_packet(0.0)
            if pkt is None:
                break
            self._push(pkt.points)
        ready = self._ready
        self._ready = None
        return ready

    def read_scan(self, timeout: float | None = None) -> list[LidarPoint] | None:
        """Block until a full 360-degree scan is assembled (None on timeout)."""
        deadline = None if timeout is None else time.time() + timeout
        while True:
            if self._ready is not None:
                ready = self._ready
                self._ready = None
                return ready
            pkt = self._read_packet(0.05)
            if pkt is not None:
                self._push(pkt.points)
                continue
            if deadline is not None and time.time() > deadline:
                return None

    def iter_scans(self):
        """Yield full scans forever (caller should break/KeyboardInterrupt)."""
        while True:
            scan = self.read_scan()
            if scan is not None:
                yield scan


def scan_to_xy(points, elevation_deg: float = 0.0):
    """Convert polar points to 2D/3D Cartesian coordinates (meters).

    Returns (xs, ys) with elevation 0; with a nonzero elevation_deg also
    returns zs as the third element.
    """
    import math

    el = math.radians(elevation_deg)
    cos_el = math.cos(el)
    xs, ys, zs = [], [], []
    for p in points:
        az = math.radians(p.angle)
        r = p.distance / 1000.0  # meters
        xs.append(r * cos_el * math.cos(az))
        ys.append(r * cos_el * math.sin(az))
        zs.append(r * math.sin(el))
    if elevation_deg == 0.0:
        return xs, ys
    return xs, ys, zs


def main(argv=None):
    """CLI smoke test: print a few scans then exit.

    Usage: ``python3 ld_lidar.py [PORT] [N]`` (PORT defaults to ``auto``).
    """
    args = sys.argv[1:] if argv is None else argv
    port = args[0] if len(args) > 0 else "auto"
    n = int(args[1]) if len(args) > 1 else 5

    try:
        with LidarScanner(port) as lidar:
            for idx, scan in enumerate(lidar.iter_scans()):
                if scan is None:
                    continue
                print(
                    f"scan {idx}: {len(scan)} pts, speed {lidar.speed_hz:.2f} Hz, "
                    f"crc ok={lidar.packets_ok} bad={lidar.packets_bad}"
                )
                for p in scan[:3]:
                    print("   ", p)
                if idx + 1 >= n:
                    break
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
