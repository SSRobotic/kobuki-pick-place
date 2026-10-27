"""Read-only Kobuki USB telemetry on Windows; never sends serial commands.

Protocol: https://kobuki.readthedocs.io/en/devel/protocol.html
Serial settings: https://github.com/kobuki-base/kobuki_core
"""

import argparse
import json
import struct
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import serial
from serial.tools import list_ports


class PacketReader:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksums = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        payloads = []
        while len(self.buffer) >= 4:
            start = self.buffer.find(b"\xaa\x55")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xAA else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 4:
                break
            size = self.buffer[2]
            if size < 3:
                del self.buffer[:1]
                continue
            frame_size = size + 4
            if len(self.buffer) < frame_size:
                break
            checksum = 0
            for byte in self.buffer[2:frame_size]:
                checksum ^= byte
            if checksum:
                self.bad_checksums += 1
                del self.buffer[:1]
                continue
            payloads.append(bytes(self.buffer[3:frame_size - 1]))
            del self.buffer[:frame_size]
        return payloads


def decode_payload(payload):
    result = {}
    offset = 0
    while offset + 2 <= len(payload):
        identifier, size = payload[offset:offset + 2]
        offset += 2
        if offset + size > len(payload):
            raise ValueError("Truncated Kobuki sensor subpacket")
        data = payload[offset:offset + size]
        offset += size
        if identifier == 1 and size == 15:
            values = struct.unpack("<HBBBHHBBBBBB", data)
            names = ("robot_timestamp_ms", "bumper", "wheel_drop", "cliff",
                     "left_encoder", "right_encoder", "left_pwm", "right_pwm",
                     "buttons", "charger", "battery_raw", "overcurrent")
            result.update(zip(names, values))
            result["battery_v"] = result.pop("battery_raw") / 10.0
            for name in ("left_pwm", "right_pwm"):
                if result[name] >= 128:
                    result[name] -= 256
        elif identifier == 4 and size == 7:
            angle, rate = struct.unpack_from("<hh", data)
            result.update(gyro_angle_raw=angle, gyro_rate_raw=rate)
    if offset != len(payload):
        raise ValueError("Incomplete Kobuki sensor subpacket header")
    return result


def ports_snapshot():
    return [{"port": p.device, "description": p.description, "hardware_id": p.hwid}
            for p in list_ports.comports()]


def monitor(port, seconds):
    parser = PacketReader()
    result = {"port": port, "baud": 115200, "mode": "receive_only",
              "bytes_received": 0, "valid_packets": 0, "malformed_payloads": 0,
              "basic_sensor_packets": 0, "latest": {}, "samples": []}
    # Set handshake lines before opening. There are deliberately no serial writes.
    connection = serial.Serial(port=None, baudrate=115200, timeout=0.2)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    connection.open()
    started = time.monotonic()
    next_sample = started
    try:
        while time.monotonic() - started < seconds:
            chunk = connection.read(min(connection.in_waiting, 4096) or 1)
            result["bytes_received"] += len(chunk)
            for payload in parser.feed(chunk):
                result["valid_packets"] += 1
                try:
                    data = decode_payload(payload)
                except ValueError:
                    result["malformed_payloads"] += 1
                    continue
                if "battery_v" in data:
                    result["basic_sensor_packets"] += 1
                result["latest"].update(data)
            now = time.monotonic()
            if now >= next_sample and result["latest"]:
                sample = {"elapsed_s": round(now - started, 3), **result["latest"]}
                result["samples"].append(sample)
                print(json.dumps(sample), flush=True)
                next_sample = now + 1.0
    finally:
        connection.close()
    result["elapsed_s"] = round(time.monotonic() - started, 3)
    result["checksum_errors"] = parser.bad_checksums
    result["passed"] = result["basic_sensor_packets"] >= 3
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="Auto-detects the serial number containing KOBUKI")
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--output", type=Path, default=Path(__file__).parent / "reports" / "latest.json")
    args = ap.parse_args()
    if not 0 < args.seconds <= 3600:
        ap.error("seconds must be between 0 and 3600")
    ports = ports_snapshot()
    report = {"checked_at": datetime.now(timezone(timedelta(hours=7))).isoformat(),
              "platform": "Windows native serial", "ports": ports}
    candidates = [p["port"] for p in ports if "KOBUKI" in p["hardware_id"].upper()]
    port = args.port or (candidates[0] if len(candidates) == 1 else None)
    print("Serial devices:", json.dumps(ports, indent=2), flush=True)
    if port:
        try:
            report["kobuki"] = monitor(port, args.seconds)
        except (serial.SerialException, OSError) as exc:
            report["kobuki"] = {"port": port, "passed": False, "error": str(exc)}
    else:
        report["kobuki"] = {"passed": False, "error": "No unique Kobuki port; use --port COMn"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("Result:", "PASS" if report["kobuki"]["passed"] else "FAIL", flush=True)
    print("Report:", args.output.resolve(), flush=True)
    return 0 if report["kobuki"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
