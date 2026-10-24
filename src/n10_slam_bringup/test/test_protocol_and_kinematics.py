import math
import struct
import unittest

from n10_slam_bringup.base_protocol import (
    CMD_TELEMETRY,
    FrameParser,
    decode_telemetry,
    encode_frame,
    encode_wheel_rpm,
)
from n10_slam_bringup.mecanum import (
    limit_wheel_rpm,
    twist_to_wheel_rpm,
    wheel_distances_to_body_delta,
)


class ProtocolAndKinematicsTest(unittest.TestCase):
    def test_wheel_command_frame(self) -> None:
        frame = encode_wheel_rpm((10, -20, 30, -40))
        self.assertEqual(frame[:3], bytes((0xAA, 0x10, 8)))
        self.assertEqual(struct.unpack('<4h', frame[3:11]), (10, -20, 30, -40))
        self.assertEqual(frame[-1], sum(frame[1:-1]) & 0xFF)

    def test_parser_resynchronises_after_noise_and_bad_checksum(self) -> None:
        good = encode_frame(0x35, b'abc')
        bad = good[:-1] + bytes((good[-1] ^ 0xFF,))
        parser = FrameParser()
        self.assertEqual(parser.feed(b'noise' + bad[:3]), [])
        frames = parser.feed(bad[3:] + b'junk' + good)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].command, 0x35)
        self.assertEqual(frames[0].payload, b'abc')

    def test_telemetry_layout_matches_stm32(self) -> None:
        payload = struct.pack(
            '<I4i4h2h2H',
            1234,
            100,
            -200,
            300,
            -400,
            10,
            -20,
            30,
            -40,
            123,
            -456,
            0x0013,
            7,
        )
        frame = encode_frame(CMD_TELEMETRY, payload)
        parsed = FrameParser().feed(frame)[0]
        telemetry = decode_telemetry(parsed.payload)
        self.assertEqual(telemetry.timestamp_ms, 1234)
        self.assertEqual(telemetry.encoder_counts, (100, -200, 300, -400))
        self.assertEqual(telemetry.wheel_rpm, (10, -20, 30, -40))
        self.assertAlmostEqual(telemetry.yaw_degrees, 12.3)
        self.assertAlmostEqual(telemetry.gyro_rate_dps, -45.6)
        self.assertEqual(telemetry.flags, 0x0013)
        self.assertEqual(telemetry.stall_events, 7)

    def test_forward_and_inverse_mecanum_motion(self) -> None:
        rpm = twist_to_wheel_rpm(0.2, 0.1, 0.5, 0.075, 0.04, 0.04)
        circumference = math.pi * 0.075
        distances = tuple(value * circumference / 60.0 for value in rpm)
        dx, dy, dyaw = wheel_distances_to_body_delta(distances, 0.04, 0.04)
        self.assertAlmostEqual(dx, 0.2)
        self.assertAlmostEqual(dy, 0.1)
        self.assertAlmostEqual(dyaw, 0.5)

    def test_rpm_limiting_preserves_motion_ratio(self) -> None:
        limited = limit_wheel_rpm((100.0, -200.0, 50.0, -25.0), 100.0)
        expected = (50.0, -100.0, 25.0, -12.5)
        for actual, wanted in zip(limited, expected):
            self.assertAlmostEqual(actual, wanted)


if __name__ == '__main__':
    unittest.main()
