#!/usr/bin/env python3
"""
visualize.py — Live (or one-shot) point-cloud view for the LD06/LD19 lidar.

Usage:
    python3 visualize.py                      # live polar animation
    python3 visualize.py --port /dev/ttyUSB1  # explicit port
    python3 visualize.py --save scan.png      # capture ONE scan to a PNG and exit
    python3 visualize.py --color-intensity --max-dist 8000

0 deg is drawn at the top, angles increasing clockwise (top-down view).
Requires: numpy, matplotlib (plus pyserial for ld_lidar).
"""
from __future__ import annotations

import argparse
import sys


def _plot_setup(fig, max_dist_mm):
    import numpy as np

    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_rlim(0, max_dist_mm)
    ax.set_facecolor("#0a0a12")
    fig.patch.set_facecolor("#0a0a12")
    ax.tick_params(colors="#888888")
    for lbl in ax.get_xticklabels() + ax.get_yticklabels():
        lbl.set_color("#aaaaaa")
    ax.grid(color="#223344", lw=0.5)
    ax.set_title("LD06/LD19 scan", color="white", pad=14)
    return ax


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="auto",
                    help="lidar serial port, or 'auto' to detect the CP210x device")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--max-dist", type=float, default=12000.0,
                    help="display range in mm")
    ap.add_argument("--color-intensity", action="store_true",
                    help="color points by reflected intensity")
    ap.add_argument("--save", metavar="PNG",
                    help="capture a single scan to a PNG file and exit")
    args = ap.parse_args()

    if args.save:
        import matplotlib
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    import numpy as np

    import ld_lidar

    fig = plt.figure(figsize=(8, 8))
    ax = _plot_setup(fig, args.max_dist)

    if args.color_intensity:
        sc = ax.scatter([], [], s=4, c=[], cmap="plasma", vmin=0, vmax=255)
    else:
        sc = ax.scatter([], [], s=4, c="lime", alpha=0.85)

    def _draw(scan):
        theta = np.radians([p.angle for p in scan])
        r = np.array([p.distance for p in scan], dtype=float)
        r = np.clip(r, 0, args.max_dist)
        sc.set_offsets(np.column_stack((theta, r)))
        if args.color_intensity:
            sc.set_array(np.array([p.intensity for p in scan], dtype=float))
        return sc

    lidar = ld_lidar.LidarScanner(args.port, args.baud)
    lidar.open()
    try:
        if args.save:
            scan = lidar.read_scan(timeout=6)
            if not scan:
                sys.exit("ERROR: no scan received within 6 s")
            _draw(scan)
            ax.set_title(
                f"LD06/LD19 scan — {len(scan)} pts @ {lidar.speed_hz:.2f} Hz",
                color="white", pad=14,
            )
            fig.savefig(args.save, dpi=150, facecolor=fig.get_facecolor())
            print(f"saved {args.save} ({len(scan)} points)")
            return

        from matplotlib.animation import FuncAnimation

        def animate(_frame):
            scan = lidar.poll_scan()
            if scan:
                _draw(scan)
                ax.set_title(
                    f"LD06/LD19 scan — {len(scan)} pts @ {lidar.speed_hz:.2f} Hz "
                    f"(crc ok={lidar.packets_ok}, bad={lidar.packets_bad})",
                    color="white", pad=14,
                )
            return (sc,)

        _ani = FuncAnimation(fig, animate, interval=60, blit=True, cache_frame_data=False)
        plt.show()
    finally:
        lidar.close()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as e:
        sys.exit(f"error: {e}")
