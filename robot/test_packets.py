"""Integrity checks for noisy/fragmented serial data, independent of hardware."""
import struct
import unittest

from kobuki_monitor import PacketReader, decode_payload


class IntegrityTests(unittest.TestCase):
    # Documented sensor layout: time, bumper/drop/cliff, two encoders, PWM,
    # button, charger, battery in tenths of a volt, overcurrent.
    basic = bytes.fromhex("01 0f 34 12 01 02 04 ff ff 00 00 ff 7f 01 00 a0 00")

    @staticmethod
    def frame(payload):
        body = bytes([len(payload)]) + payload
        checksum = 0
        for byte in body:
            checksum ^= byte
        return b"\xaa\x55" + body + bytes([checksum])

    def test_fragmented_frame_with_leading_noise(self):
        frame = self.frame(self.basic)
        parser = PacketReader()
        result = []
        for byte in b"noise" + frame:
            result.extend(parser.feed(bytes([byte])))
        self.assertEqual(result, [self.basic])

    def test_bad_checksum_is_rejected_and_next_frame_recovers(self):
        frame = bytearray(self.frame(self.basic))
        frame[-1] ^= 1
        parser = PacketReader()
        self.assertEqual(parser.feed(frame + self.frame(self.basic)), [self.basic])
        self.assertEqual(parser.bad_checksums, 1)

    def test_sensor_values_keep_wraparound_signed_pwm_and_voltage(self):
        data = decode_payload(self.basic)
        self.assertEqual(data["robot_timestamp_ms"], 0x1234)
        self.assertEqual(data["left_encoder"], 65535)
        self.assertEqual(data["left_pwm"], -1)
        self.assertEqual(data["right_pwm"], 127)
        self.assertEqual(data["battery_v"], 16.0)
        self.assertEqual((data["bumper"], data["wheel_drop"], data["cliff"]), (1, 2, 4))

    def test_truncated_sensor_is_rejected(self):
        with self.assertRaises(ValueError):
            decode_payload(self.basic[:-1])

    def test_gyro_is_signed(self):
        data = decode_payload(b"\x04\x07" + struct.pack("<hhBBB", -100, -20, 0, 0, 0))
        self.assertEqual(data["gyro_angle_raw"], -100)
        self.assertEqual(data["gyro_rate_raw"], -20)


if __name__ == "__main__":
    unittest.main()
