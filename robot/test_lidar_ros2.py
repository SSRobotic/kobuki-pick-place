"""Offline geometry and integrity checks; never open a hardware serial port."""
import math
import unittest

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan

from lidar_ros2 import ScanAssembler, make_scan


class ScanMessageTests(unittest.TestCase):
    def test_axes_wrap_and_nearest_collision(self):
        scan = make_scan([(0, 2), (0.1, 1), (90, 3), (180, 4),
                          (360, 5), (270, 6)], 1_000_000_123, 0.25)
        self.assertEqual(len(scan.ranges), 720)
        self.assertEqual(scan.ranges[360], 1)
        self.assertEqual(scan.ranges[540], 3)
        self.assertEqual(scan.ranges[0], 4)
        self.assertEqual(scan.ranges[180], 6)
        self.assertEqual(scan.header.stamp.sec, 1)
        self.assertEqual(scan.header.stamp.nanosec, 123)
        self.assertAlmostEqual(scan.angle_max, scan.angle_min + 719 * scan.angle_increment, places=5)

    def test_invalid_returns_are_absent_and_ros_serialization_works(self):
        scan = make_scan([(0, 0), (10, 0.05), (20, 9), (30, math.nan),
                          (math.nan, 1), (90, 2)], 42, 0.25)
        restored = deserialize_message(serialize_message(scan), LaserScan)
        self.assertEqual(restored.header.frame_id, "laser")
        self.assertEqual(sum(math.isfinite(d) for d in restored.ranges), 1)
        self.assertEqual(restored.ranges[540], 2)
        self.assertEqual(restored.time_increment, 0)
        self.assertEqual(len(restored.intensities), 0)

    def test_partial_or_corrupted_revolutions_are_not_published(self):
        assembler = ScanAssembler()
        self.assertEqual(assembler.feed([{"start": False, "points": [(1, 1)]}], 10, 0, 0), [])
        self.assertEqual(assembler.feed([{"start": True, "points": [(2, 1)]}], 20, 1, 0), [])
        # A checksum error anywhere in the revolution invalidates that scan.
        self.assertEqual(assembler.feed([{"start": True, "points": [(3, 1)]}], 30, 1.25, 1), [])
        self.assertEqual(assembler.feed([{"start": True, "points": [(4, 1)]}], 40, 1.5, 1), [])
        scans = assembler.feed([{"start": True, "points": [(5, 1)]}], 50, 1.75, 1)
        self.assertEqual(scans, [([(4, 1)], 40, 0.25)])


if __name__ == "__main__":
    unittest.main()
