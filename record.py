#!/usr/bin/env python3
"""
record.py — Record lidar scans to CSV and/or PLY point-cloud files.

A 2D lidar gives one flat 360-degree ring per scan. To build a genuine
**3D point cloud**, mount the lidar on a tilting/rotating stage so each scan is
taken at a different elevation. Pass --tilt-start / --tilt-end and the recorder
assumes the stage sweeps uniformly across the recorded scans, converting each
point to (x, y, z).

Usage:
    python3 record.py --scans 1 --output room                        # 2D, CSV
    python3 record.py --scans 100 --format ply --output sweep \
        --tilt-start -45 --tilt-end 45                               # 3D sweep
    python3 record.py --scans 10 --format both --output scan
"""
from __future__ import annotations

import argparse
import csv
import math
import sys

import ld_lidar


def to_xyz(point, elevation_deg):
    """Polar point + scan elevation -> Cartesian (x, y, z) in meters."""
    az = math.radians(point.angle)
    el = math.radians(elevation_deg)
    d = point.distance / 1000.0
    x = d * math.cos(el) * math.cos(az)
    y = d * math.cos(el) * math.sin(az)
    z = d * math.sin(el)
    return x, y, z


def write_csv(path, scans):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scan_index", "angle_deg", "distance_mm", "intensity"])
        for si, scan in enumerate(scans):
            for p in scan:
                w.writerow([si, f"{p.angle:.3f}", p.distance, p.intensity])


def write_ply(path, scans, elevations):
    total = sum(len(s) for s in scans)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {total}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar intensity\n")
        f.write("end_header\n")
        for scan, el in zip(scans, elevations):
            for p in scan:
                x, y, z = to_xyz(p, el)
                f.write(f"{x:.4f} {y:.4f} {z:.4f} {p.intensity}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="auto",
                    help="lidar serial port, or 'auto' to detect the CP210x device")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--scans", type=int, default=1,
                    help="number of full 360-degree scans to record")
    ap.add_argument("--output", default="scan", help="output base name (no extension)")
    ap.add_argument("--format", default="csv", choices=["csv", "ply", "both"])
    ap.add_argument("--tilt-start", type=float, default=0.0,
                    help="scan-plane elevation (deg) at the first scan")
    ap.add_argument("--tilt-end", type=float, default=0.0,
                    help="scan-plane elevation (deg) at the last scan (3D sweep)")
    args = ap.parse_args()

    with ld_lidar.LidarScanner(args.port, args.baud) as lidar:
        scans = []
        for i in range(args.scans):
            print(f"recording scan {i + 1}/{args.scans} ...", end="\r")
            scan = lidar.read_scan(timeout=5)
            if scan is None:
                print("\nWARNING: scan timeout, stopping early")
                break
            scans.append(scan)
        print()
        if not scans:
            sys.exit("ERROR: no scans captured")

    n = len(scans)
    if args.tilt_start != args.tilt_end and n > 1:
        elevations = [
            args.tilt_start + (args.tilt_end - args.tilt_start) * i / (n - 1)
            for i in range(n)
        ]
    else:
        elevations = [args.tilt_start] * n

    total = sum(len(s) for s in scans)
    if args.format in ("csv", "both"):
        path = args.output + ".csv"
        write_csv(path, scans)
        print(f"wrote {path} ({total} points)")
    if args.format in ("ply", "both"):
        path = args.output + ".ply"
        write_ply(path, scans, elevations)
        dims = "3D" if any(e != 0.0 for e in elevations) else "2D"
        print(f"wrote {path} ({total} points, {dims})")


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as e:
        sys.exit(f"error: {e}")
