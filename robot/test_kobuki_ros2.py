"""Odometry geometry, clock/counter continuity, and ROS message integrity."""
import math
import unittest

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message

from kobuki_ros2 import EncoderOdometry, TICK_METERS, WHEEL_SEPARATION, make_odometry


class OdometryTests(unittest.TestCase):
    def test_first_reading_establishes_origin_without_encoder_jump(self):
        state = EncoderOdometry()
        self.assertTrue(state.update(32000, 62000, 45000))
        self.assertEqual((state.x, state.y, state.yaw), (0, 0, 0))

    def test_equal_wheel_steps_translate_forward_and_backward(self):
        state = EncoderOdometry()
        state.update(100, 1000, 2000)
        state.update(120, 1100, 2100)
        self.assertAlmostEqual(state.x, 100 * TICK_METERS)
        self.assertEqual(state.y, 0)
        self.assertAlmostEqual(state.linear, 100 * TICK_METERS / 0.02)
        state.update(140, 1000, 2000)
        self.assertAlmostEqual(state.x, 0)
        self.assertLess(state.linear, 0)

    def test_opposite_steps_rotate_without_translation(self):
        state = EncoderOdometry()
        state.update(100, 1000, 1000)
        state.update(120, 900, 1100)
        angle = 200 * TICK_METERS / WHEEL_SEPARATION
        self.assertEqual((state.x, state.y), (0, 0))
        self.assertAlmostEqual(state.yaw, angle)
        self.assertAlmostEqual(state.angular, angle / 0.02)

    def test_arc_follows_exact_planar_turn(self):
        state = EncoderOdometry()
        state.update(100, 1000, 1000)
        state.update(200, 1100, 1300)
        turn = 200 * TICK_METERS / WHEEL_SEPARATION
        radius = 200 * TICK_METERS / turn
        self.assertAlmostEqual(state.x, radius * math.sin(turn))
        self.assertAlmostEqual(state.y, radius * (1 - math.cos(turn)))

    def test_clock_and_encoder_wrap_preserve_small_forward_motion(self):
        state = EncoderOdometry()
        state.update(65530, 65530, 65530)
        self.assertTrue(state.update(14, 4, 4))
        self.assertAlmostEqual(state.x, 10 * TICK_METERS)
        self.assertAlmostEqual(state.linear, 10 * TICK_METERS / 0.02)

    def test_duplicate_long_gap_and_implausible_jump_do_not_move_pose(self):
        state = EncoderOdometry()
        state.update(100, 100, 100)
        self.assertFalse(state.update(100, 100, 100))
        self.assertFalse(state.update(700, 0, 0))
        self.assertFalse(state.update(720, 20000, 20000))
        self.assertEqual((state.x, state.y, state.yaw), (0, 0, 0))
        self.assertEqual(state.rebases, 2)
        self.assertTrue(state.update(740, 20001, 20001))
        self.assertAlmostEqual(state.x, TICK_METERS)

    def test_tf_and_serialized_odometry_share_pose_stamp_and_frames(self):
        state = EncoderOdometry()
        state.update(100, 1000, 1000)
        state.update(200, 1100, 1300)
        msg, transform = make_odometry(state, Time(sec=12, nanosec=34))
        restored = deserialize_message(serialize_message(msg), Odometry)
        self.assertEqual(restored.header.frame_id, "odom")
        self.assertEqual(restored.child_frame_id, "base_link")
        self.assertEqual(transform.header, restored.header)
        self.assertEqual(transform.transform.rotation, restored.pose.pose.orientation)
        self.assertEqual(transform.transform.translation.x, restored.pose.pose.position.x)
        self.assertAlmostEqual(transform.transform.rotation.z ** 2 + transform.transform.rotation.w ** 2, 1)
        self.assertGreater(restored.pose.covariance[14], 100)


if __name__ == "__main__":
    unittest.main()
