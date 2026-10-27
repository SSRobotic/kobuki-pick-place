"""Publish the tested Windows YDLIDAR X2 serial stream as /scan.

Run this with the RoboStack ROS environment, not the diagnostic-only venv.
This displays scans in the sensor's local frame. Mounting calibration and
odometry are still needed before mapping. Acquisition stamps approximate USB
receipt times; resampled rays have no individual time measurements.
"""
import argparse
import math
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from serial.tools import list_ports

from lidar_monitor import X2PacketReader


def make_scan(points, stamp_ns, scan_time, frame="laser", bins=720,
              range_min=0.12, range_max=8.0, mirror=False):
    """Resample corrected angles into ROS's uniform angular grid.

    Keep the nearest valid return when two rays land in one bin. Missing,
    zero, and out-of-range returns remain infinity, never obstacle points.
    Default angle direction matches the SDK with Inverted=False/Reversion=False.
    """
    msg = LaserScan()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
    msg.header.frame_id = frame
    msg.angle_min = -math.pi
    msg.angle_increment = math.tau / bins
    msg.angle_max = msg.angle_min + (bins - 1) * msg.angle_increment
    msg.scan_time = float(scan_time)
    # Angular binning reorders samples; do not invent per-ray timing.
    msg.time_increment = 0.0
    msg.range_min, msg.range_max = float(range_min), float(range_max)
    ranges = [math.inf] * bins
    for degrees, distance in points:
        if not math.isfinite(degrees) or not math.isfinite(distance):
            continue
        if not range_min <= distance <= range_max:
            continue
        if mirror:
            degrees = -degrees
        angle = (math.radians(degrees) + math.pi) % math.tau - math.pi
        index = int(math.floor((angle + math.pi) / msg.angle_increment + 0.5)) % bins
        ranges[index] = min(ranges[index], float(distance))
    msg.ranges = ranges
    return msg


class ScanAssembler:
    """Only emit revolutions enclosed by two valid start markers."""
    def __init__(self):
        self.points = []
        self.stamp_ns = None
        self.started_at = None
        self.checksum_errors = 0

    def feed(self, packets, stamp_ns, now, checksum_errors):
        if checksum_errors != self.checksum_errors:
            # The streaming parser reports errors per read batch, not per
            # packet. Drop that whole batch and await a fresh start marker.
            self.points = []
            self.stamp_ns = None
            self.checksum_errors = checksum_errors
            return []
        scans = []
        for packet in packets:
            if packet["start"]:
                if (self.stamp_ns is not None and self.points
                        and checksum_errors == self.checksum_errors):
                    duration = now - self.started_at
                    if 0.02 <= duration <= 2.0:
                        scans.append((self.points, self.stamp_ns, duration))
                self.points = []
                self.stamp_ns = stamp_ns
                self.started_at = now
                self.checksum_errors = checksum_errors
            if self.stamp_ns is not None:
                self.points.extend(packet["points"])
                # Bound memory if the USB stream loses revolution markers.
                if len(self.points) > 10000:
                    self.points = []
                    self.stamp_ns = None
        return scans


class X2ScanNode(Node):
    def __init__(self, port, frame, mirror=False):
        super().__init__("windows_ydlidar_x2")
        self.frame = frame
        self.mirror = mirror
        self.publisher = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.parser = X2PacketReader()
        self.assembler = ScanAssembler()
        self.scans = 0
        self.last_report = time.monotonic()
        self.connection = serial.Serial(port=None, baudrate=115200, timeout=0)
        # These handshake settings produced valid X2 scans on this adapter.
        self.connection.dtr = False
        self.connection.rts = False
        self.connection.port = port
        try:
            self.connection.open()
        except serial.SerialException:
            self.destroy_node()
            raise
        self.timer = self.create_timer(0.01, self.read_serial)
        self.get_logger().info(f"Reading {port} at 115200; publishing /scan, frame={frame}, mirror={mirror}")

    def read_serial(self):
        available = self.connection.in_waiting
        if available:
            packets = self.parser.feed(self.connection.read(min(available, 4096)))
            now = time.monotonic()
            stamp_ns = self.get_clock().now().nanoseconds
            for points, stamp, duration in self.assembler.feed(
                    packets, stamp_ns, now, self.parser.bad_checksums):
                self.publisher.publish(make_scan(points, stamp, duration, self.frame, mirror=self.mirror))
                self.scans += 1
                if self.scans == 1:
                    valid = sum(0.12 <= d <= 8.0 for _, d in points)
                    self.get_logger().info(f"First complete scan: {valid} valid returns. RViz: Fixed Frame=laser, LaserScan topic=/scan, Reliability=Best Effort")
        now = time.monotonic()
        if now - self.last_report >= 5:
            if self.scans:
                self.get_logger().info(f"Published {self.scans} scans; checksum errors={self.parser.bad_checksums}")
            else:
                self.get_logger().warning("No complete scan yet. Check LiDAR USB connection and close other programs using its COM port.")
            self.last_report = now

    def close(self):
        self.connection.close()
        self.destroy_node()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="COM port; auto-selects a unique CP210x adapter")
    ap.add_argument("--frame", default="laser")
    ap.add_argument("--mirror", action="store_true", help="Flip angle direction (clockwise X2 -> ROS counter-clockwise)")
    args, ros_args = ap.parse_known_args()
    candidates = [p.device for p in list_ports.comports() if p.vid == 0x10C4 and p.pid == 0xEA60]
    port = args.port or (candidates[0] if len(candidates) == 1 else None)
    if not port:
        ap.error("No unique LiDAR adapter. Specify --port COMn.")
    rclpy.init(args=ros_args)
    node = None
    code = 0
    try:
        node = X2ScanNode(port, args.frame, args.mirror)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"LiDAR serial error: {exc}", flush=True)
        code = 1
    finally:
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
