import math
import struct
import unittest

from feedback_probe import FeedbackParser, crc16


def make_frame(seq=7, stamp_ms=12345, quaternion=(1.0, 0.0, 0.0, 0.0),
               yaw=0.25, pitch=-0.125, mode=2):
    body = b"\xb5" + struct.pack("<II6fB", seq, stamp_ms, *quaternion, yaw, pitch, mode)
    return body + struct.pack("<H", crc16(body)) + b"\x5b\x5a"


class FeedbackProbeTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x6F91)

    def test_layout_and_units(self):
        wire = make_frame()
        self.assertEqual(len(wire), 38)
        frame = FeedbackParser().feed(wire)[0]
        self.assertEqual(frame["sequence"], 7)
        self.assertEqual(frame["mcu_timestamp_ms"], 12345)
        self.assertEqual(frame["quaternion_wxyz"], [1, 0, 0, 0])
        self.assertEqual(frame["mode"], 2)
        self.assertAlmostEqual(frame["yaw_deg"], math.degrees(0.25))
        self.assertAlmostEqual(frame["pitch_rad"], -0.125)
        self.assertEqual(frame["warnings"], [])

    def test_every_split_point(self):
        wire = make_frame()
        for split in range(1, len(wire)):
            with self.subTest(split=split):
                parser = FeedbackParser()
                self.assertEqual(parser.feed(wire[:split]), [])
                self.assertEqual(len(parser.feed(wire[split:])), 1)

    def test_noise_and_concatenated_frames(self):
        parser = FeedbackParser()
        frames = parser.feed(b"noise" + make_frame(seq=1) + make_frame(seq=2))
        self.assertEqual([frame["sequence"] for frame in frames], [1, 2])
        self.assertEqual(parser.discarded_bytes, 5)

    def test_bad_crc_resynchronizes(self):
        wire = bytearray(make_frame())
        wire[1] ^= 1
        parser = FeedbackParser()
        frames = parser.feed(wire + make_frame(seq=8))
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual([frame["sequence"] for frame in frames], [8])

    def test_bad_tail_resynchronizes(self):
        wire = make_frame()[:-1] + b"\x00"
        parser = FeedbackParser()
        frames = parser.feed(wire + make_frame(seq=8))
        self.assertEqual(parser.tail_errors, 1)
        self.assertEqual([frame["sequence"] for frame in frames], [8])

    def test_zero_quaternion_is_not_ready(self):
        frame = FeedbackParser().feed(make_frame(quaternion=(0, 0, 0, 0)))[0]
        self.assertEqual(frame["quaternion_norm"], 0)
        self.assertIn("quaternion_not_unit", frame["warnings"])

    def test_nonfinite_pose_is_not_usable(self):
        frame = FeedbackParser().feed(make_frame(yaw=float("nan")))[0]
        self.assertIsNone(frame["yaw_rad"])
        self.assertIn("nonfinite_pose", frame["warnings"])

    def test_embedded_header_is_payload(self):
        frame = FeedbackParser().feed(make_frame(seq=0xB5))[0]
        self.assertEqual(frame["sequence"], 0xB5)


if __name__ == "__main__":
    unittest.main()
