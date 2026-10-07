# Lidar Linux Scanner D600

Linux driver and command-line tools for the **Waveshare D600 LiDAR scanner**
(LDRobot LD06/LD19 serial protocol family). Pure Python, no compilation — read
the lidar over its USB-serial port, visualize the point cloud, record it to
CSV/PLY, and relay it over a UART radio link (e.g. the SX1262 LoRa HAT).

> The vendor ships a Windows-only desktop viewer (`ld_desktop`) with no source
> code. This project re-implements the wire protocol from scratch so the
> D600 / LD06 / LD19 can be used on Linux, Raspberry Pi, and other SBCs.

## Features

- **Auto-detects** the lidar's CP210x USB-serial port — no `--port` needed.
- CRC-checked packet parsing and full 360° scan assembly.
- Live polar point-cloud view, or capture a single scan to PNG.
- Record scans to **CSV** and **PLY** (2D, or 3D via a tilting mount).
- Forward scans over a transparent UART radio link (LoRa).

## Hardware

- **Lidar:** Waveshare D600 (also compatible with LD06 / LD19 — identical protocol).
- **Interface:** CP2102 USB-UART bridge (USB VID `10c4`), **230400 baud**.
- On Linux it enumerates as `/dev/ttyUSB*`; the stable symlink is
  `/dev/serial/by-id/usb-*CP2102*`.

## Install

Debian / Ubuntu / Raspberry Pi OS (system packages):

```bash
sudo apt install python3-serial                     # required
sudo apt install python3-numpy python3-matplotlib   # for visualize.py
```

or with pip:

```bash
pip install pyserial
pip install numpy matplotlib                        # for visualize.py
```

or install this project itself (editable, from a clone):

```bash
git clone https://github.com/<your-account>/lidar-linux-scanner-d600.git
cd lidar-linux-scanner-d600
pip install -e .              # core (installs pyserial)
pip install -e ".[visual]"    # + numpy/matplotlib
```

Serial access: add your user to the `dialout` group, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

## Usage

```bash
# smoke test: print a few scans (auto-detects the port)
python3 ld_lidar.py

# live point-cloud view
python3 visualize.py

# capture one scan to a PNG (works headless)
python3 visualize.py --save out.png
python3 visualize.py --color-intensity --max-dist 8000

# record to CSV / PLY
python3 record.py --scans 1 --output room

# 3D point cloud from a tilting mount
python3 record.py --scans 100 --format ply --output sweep \
    --tilt-start -45 --tilt-end 45

# relay over a LoRa HAT
python3 relay_lora.py --lora-port /dev/ttyS0
python3 relay_lora.py --lora-port /dev/ttyUSB2 --mode full --hz 2
```

Every tool accepts `--port <device>` to override auto-detection (useful when
more than one CP210x device is attached).

## Wire protocol

47-byte packets, little-endian, continuous stream:

| offset | size | field |
|--------|------|-------|
| 0  | 1  | header `0x54` |
| 1  | 1  | `0x2C` |
| 2  | 2  | speed (deg/s) |
| 4  | 2  | start angle (0.01°) |
| 6  | 36 | 12 × (distance u16 mm, intensity u8) |
| 42 | 2  | end angle (0.01°) |
| 44 | 2  | timestamp (ms) |
| 46 | 1  | CRC-8 |

- **CRC-8:** MSB-first, polynomial `0x4D`, init `0x00`, no reflection/XOR
  (full polynomial `0x14D`), computed over bytes 0–45.
- **Point angle** (degrees), point `i` of 12:
  ```
  diff  = (end + 36000 - start) % 36000
  angle = start/100 + i * (diff / 11) / 100
  ```
- A full revolution is ~45 packets / ~500 points (~4500 points/s at ~10 Hz).

## 3D scanning

The D600 / LD-series sensor emits one flat 360° ring per scan. To build a true
3D point cloud, mount it on a tilting/rotating stage and use
`record.py --tilt-start/--tilt-end`; it assumes a uniform sweep and emits
`(x, y, z)` coordinates. The PLY output opens in CloudCompare, MeshLab, or
`open3d`.

## LoRa relay

`relay_lora.py` writes framed packets into a transparent UART radio link:

- `--mode summary` (default) — closest obstacle per angular sector; low
  bandwidth, fits a 9600-baud LoRa link.
- `--mode full` — complete point cloud; needs a high-speed link (~22.5 kbit/s
  at 10 Hz).
- `--mode json` — human-readable, for debugging.

The exact frame format is documented in `relay_lora.py`.

## License

[GNU GPL v3.0 or later](LICENSE).
