"""Receive YDLIDAR X2 scans directly from its Windows USB serial adapter.

Packet checksum and triangle correction follow the manufacturer's protocol:
https://github.com/YDLIDAR/YDLidar-SDK/blob/master/src/YDlidarDriver.cpp
The output coordinates are in the LiDAR's own frame, not a SLAM map.
"""
import argparse
import csv
import json
import math
import struct
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import serial
from serial.tools import list_ports


class X2PacketReader:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksums = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        packets = []
        while len(self.buffer) >= 10:
            start = self.buffer.find(b"\xaa\x55")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xAA else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 10:
                break
            count = self.buffer[3]
            first, last = struct.unpack_from("<HH", self.buffer, 4)
            if not count or not (first & 1 and last & 1):
                del self.buffer[:1]
                continue
            size = 10 + 2 * count
            if len(self.buffer) < size:
                break
            words = struct.unpack_from(f"<{size // 2}H", self.buffer)
            checksum = 0
            for index, word in enumerate(words):
                if index != 4:
                    checksum ^= word
            if checksum != words[4]:
                self.bad_checksums += 1
                del self.buffer[:1]
                continue
            ct = self.buffer[2]
            first_deg, last_deg = (first >> 1) / 64, (last >> 1) / 64
            span = (last_deg - first_deg) % 360 if count > 1 else 0
            points = []
            for index, raw_distance in enumerate(words[5:]):
                distance_mm = raw_distance / 4.0
                angle = first_deg + (span * index / (count - 1) if count > 1 else 0)
                if distance_mm:
                    correction = math.degrees(math.atan(
                        21.8 * (155.3 - distance_mm) / (155.3 * distance_mm)))
                    angle += correction
                points.append((angle % 360, distance_mm / 1000.0))
            packets.append({"start": bool(ct & 1), "points": points})
            del self.buffer[:size]
        return packets


def write_scan_svg(points, path):
    valid = [(a, d) for a, d in points if d > 0]
    extent = max(1, math.ceil(max((d for _, d in valid), default=1)))
    cx, cy, radius = 360, 370, 290
    scale = radius / extent
    elements = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="720" height="740" viewBox="0 0 720 740">',
        '<rect width="720" height="740" fill="#101824"/>',
        '<g font-family="Segoe UI, sans-serif" fill="#e7eff9">',
        '<text x="32" y="42" font-size="24">YDLIDAR X2 - real sensor scan</text>',
        '<text x="32" y="70" font-size="14" fill="#a8bbd0">Local LiDAR frame; no mounting calibration; not a SLAM map.</text>',
    ]
    for distance in range(1, extent + 1):
        rr = distance * scale
        elements.append(f'<circle cx="{cx}" cy="{cy}" r="{rr:.2f}" fill="none" stroke="#344356"/>')
        elements.append(f'<text x="{cx + 5}" y="{cy - rr - 4:.2f}" font-size="12">{distance} m</text>')
    elements.extend([
        '<path d="M70 370H650 M360 80V660" stroke="#344356"/>',
        '<text x="655" y="375" font-size="12">0 deg</text>',
        '<text x="325" y="685" font-size="12">270 deg</text>',
    ])
    for angle, distance in valid:
        aa = math.radians(angle)
        xx, yy = cx + distance * math.cos(aa) * scale, cy - distance * math.sin(aa) * scale
        elements.append(f'<circle cx="{xx:.2f}" cy="{yy:.2f}" r="2" fill="#66e8c3"/>')
    elements.extend([
        f'<circle cx="{cx}" cy="{cy}" r="5" fill="#ffbb60"/>',
        f'<text x="32" y="720" font-size="14">{len(valid)} nonzero returns in one revolution</text>',
        '</g></svg>',
    ])
    path.write_text("\n".join(elements), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="Auto-selects a unique CP210x adapter")
    ap.add_argument("--seconds", type=float, default=5)
    args = ap.parse_args()
    if not 0 < args.seconds <= 3600:
        ap.error("seconds must be between 0 and 3600")
    candidates = [p.device for p in list_ports.comports() if p.vid == 0x10C4 and p.pid == 0xEA60]
    port = args.port or (candidates[0] if len(candidates) == 1 else None)
    if not port:
        ap.error("No unique CP210x adapter; specify --port COMn")
    parser = X2PacketReader()
    result = {"checked_at": datetime.now(timezone(timedelta(hours=7))).isoformat(),
              "port": port, "baud": 115200, "bytes_received": 0,
              "valid_packets": 0, "points_received": 0,
              "nonzero_returns": 0, "complete_revolutions": 0,
              "frame": "uncalibrated_local_lidar", "ros_enabled": False}
    scan = []
    last_scan = []
    started_scan = False
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    # False DTR is the setting that produced the observed X2 data on this adapter.
    connection.dtr = False
    connection.rts = False
    connection.port = port
    connection.open()
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            chunk = connection.read(min(connection.in_waiting, 4096) or 1)
            result["bytes_received"] += len(chunk)
            for packet in parser.feed(chunk):
                result["valid_packets"] += 1
                result["points_received"] += len(packet["points"])
                result["nonzero_returns"] += sum(d > 0 for _, d in packet["points"])
                if packet["start"]:
                    if started_scan and scan:
                        last_scan = scan
                        result["complete_revolutions"] += 1
                    scan = []
                    started_scan = True
                if started_scan:
                    scan.extend(packet["points"])
    finally:
        connection.close()
    result["elapsed_s"] = round(time.monotonic() - started, 3)
    result["checksum_errors"] = parser.bad_checksums
    result["scan_points"] = len(last_scan)
    result["scan_nonzero_returns"] = sum(d > 0 for _, d in last_scan)
    result["passed"] = result["complete_revolutions"] >= 2 and result["scan_nonzero_returns"] >= 10
    destination = Path(__file__).parent / "reports"
    destination.mkdir(parents=True, exist_ok=True)
    with (destination / "lidar_scan.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["angle_deg", "distance_m"])
        writer.writerows(last_scan)
    if last_scan:
        write_scan_svg(last_scan, destination / "lidar_scan.svg")
    else:
        (destination / "lidar_scan.svg").unlink(missing_ok=True)
    (destination / "lidar_latest.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
    print("Result:", "PASS" if result["passed"] else "FAIL", flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
