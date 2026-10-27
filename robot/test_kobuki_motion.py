"""Motor protocol and timeout checks with no connection to the physical robot."""
import math
import struct
import unittest
from unittest.mock import Mock, patch

import rclpy
from geometry_msgs.msg import Twist

from kobuki_motion import MotionController, STOP_FRAME, command_frame
from kobuki_ros2 import KobukiOdometryNode


GOOD_TELEMETRY = dict(bumper=0, wheel_drop=0, cliff=0, overcurrent=0, charger=0)


class MotorProtocolTests(unittest.TestCase):
    def test_known_forward_stop_and_rotation_frames(self):
        self.assertEqual(command_frame(0.1, 0), bytes.fromhex("aa 55 06 01 04 64 00 00 00 67"))
        self.assertEqual(STOP_FRAME, bytes.fromhex("aa 55 06 01 04 00 00 00 00 03"))
        self.assertEqual(command_frame(0, 0.4), bytes.fromhex("aa 55 06 01 04 2e 00 01 00 2c"))
        self.assertEqual(struct.unpack_from("<hh", command_frame(0, -0.4), 5), (-46, 1))

    def test_arcs_and_reverse_use_outer_wheel_speed(self):
        self.assertEqual(struct.unpack_from("<hh", command_frame(0.1, 0.4), 5), (146, 250))
        self.assertEqual(struct.unpack_from("<hh", command_frame(0.1, -0.4), 5), (146, -250))
        self.assertEqual(struct.unpack_from("<hh", command_frame(-0.1, 0.4), 5), (-146, -250))
        self.assertEqual(struct.unpack_from("<hh", command_frame(-0.1, 0), 5), (-100, 0))

    def test_timeout_stops_and_does_not_replay_old_command_after_recovery(self):
        ctl = MotionController()
        self.assertFalse(ctl.accept(0.1, 0, 0))
        ctl.update_telemetry(GOOD_TELEMETRY, 0)
        self.assertTrue(ctl.accept(0.1, 0, 0))
        self.assertNotEqual(ctl.frame(0.1), STOP_FRAME)
        ctl.update_telemetry(GOOD_TELEMETRY, 0.39)
        self.assertEqual(ctl.frame(0.41), STOP_FRAME)
        self.assertTrue(ctl.accept(0.1, 0, 0.42))
        self.assertEqual(ctl.frame(0.7), STOP_FRAME)
        ctl.update_telemetry(GOOD_TELEMETRY, 0.71)
        self.assertEqual(ctl.frame(0.71), STOP_FRAME)

    def test_faults_stop_and_require_a_fresh_command(self):
        for name in GOOD_TELEMETRY:
            with self.subTest(flag=name):
                ctl = MotionController()
                ctl.update_telemetry(GOOD_TELEMETRY, 0)
                ctl.accept(0.1, 0, 0)
                ctl.update_telemetry({**GOOD_TELEMETRY, name: 1}, 0.1)
                self.assertEqual(ctl.frame(0.1), STOP_FRAME)
                self.assertFalse(ctl.accept(0.1, 0, 0.1))
                ctl.update_telemetry(GOOD_TELEMETRY, 0.2)
                self.assertEqual(ctl.frame(0.2), STOP_FRAME)
                self.assertTrue(ctl.accept(0.1, 0, 0.2))

    def test_limits_and_invalid_input(self):
        ctl = MotionController()
        ctl.update_telemetry(GOOD_TELEMETRY, 0)
        self.assertTrue(ctl.accept(2, -5, 0))
        self.assertEqual((ctl.linear, ctl.angular), (0.12, -0.6))
        self.assertFalse(ctl.accept(math.nan, 0, 0.1))
        self.assertEqual(ctl.frame(0.1), STOP_FRAME)
        with self.assertRaises(ValueError):
            command_frame(0, math.inf)


class FakeSerialNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def make_connection(self):
        connection = Mock()
        connection.is_open = True
        connection.in_waiting = 0
        connection.write.side_effect = lambda data: len(data)
        return connection

    def test_receive_only_mode_never_writes_to_port(self):
        connection = self.make_connection()
        with patch('kobuki_ros2.serial.Serial', return_value=connection):
            node = KobukiOdometryNode('FAKE_PORT')
            node.read_serial()
            node.close()
        connection.write.assert_not_called()

    def test_keyboard_callback_drive_and_shutdown_use_mock_port_only(self):
        import time
        connection = self.make_connection()
        with patch('kobuki_ros2.serial.Serial', return_value=connection):
            node = KobukiOdometryNode('FAKE_PORT', enable_motion=True)
            node.motion.update_telemetry(GOOD_TELEMETRY, time.monotonic())
            msg = Twist()
            msg.linear.x = 0.08
            node.command_received(msg)
            node.send_motion()
            self.assertEqual(connection.write.call_args.args[0], command_frame(0.08, 0))
            node.close()
        self.assertEqual([call.args[0] for call in connection.write.call_args_list[-3:]], [STOP_FRAME] * 3)


if __name__ == '__main__':
    unittest.main()
