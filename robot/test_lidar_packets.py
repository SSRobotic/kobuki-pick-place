import math
import struct
import unittest

from lidar_monitor import X2PacketReader


class X2IntegrityTests(unittest.TestCase):
    @staticmethod
    def frame(first=359, last=1, distances=(4000, 4000, 4000), ct=1):
        first_word, last_word = round(first * 64) * 2 + 1, round(last * 64) * 2 + 1
        count = len(distances)
        checksum = 0x55AA ^ (ct | count << 8) ^ first_word ^ last_word
        for distance in distances:
            checksum ^= distance
        return struct.pack("<HBBHHH", 0x55AA, ct, count, first_word, last_word, checksum) + struct.pack(f"<{count}H", *distances)

    def test_angle_wrap_uses_short_forward_arc_and_millimeter_scaling(self):
        packet = X2PacketReader().feed(self.frame())[0]
        correction = math.degrees(math.atan(21.8 * (155.3 - 1000) / (155.3 * 1000)))
        self.assertTrue(packet["start"])
        self.assertEqual([d for _, d in packet["points"]], [1, 1, 1])
        for (actual, _), angle in zip(packet["points"], (359, 360, 361)):
            self.assertAlmostEqual(actual, (angle + correction) % 360)

    def test_byte_fragments_and_leading_noise(self):
        parser = X2PacketReader()
        result = []
        for byte in b"noise" + self.frame(distances=(0, 4000, 8000)):
            result.extend(parser.feed(bytes([byte])))
        self.assertEqual(len(result), 1)
        self.assertEqual([d for _, d in result[0]["points"]], [0, 1, 2])

    def test_corruption_does_not_produce_points_and_resyncs(self):
        corrupt = bytearray(self.frame())
        corrupt[-1] ^= 1
        parser = X2PacketReader()
        self.assertEqual(len(parser.feed(corrupt + self.frame())), 1)
        self.assertEqual(parser.bad_checksums, 1)


if __name__ == "__main__":
    unittest.main()
