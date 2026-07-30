import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from lxs1_uart import VISION_FRAME_LENGTH, VisionLineSender


class FakeUart:
    def __init__(self):
        self.data = None

    def write(self, data):
        self.data = bytes(data)
        return len(data)


def read_i16_le(data, offset):
    value = data[offset] | (data[offset + 1] << 8)
    return value - 0x10000 if value & 0x8000 else value


class VisionLineSenderTest(unittest.TestCase):
    def test_exact_frame_layout(self):
        uart = FakeUart()
        sender = VisionLineSender(uart, 0x05, 0x04)
        sender.update(1, 2, 876, -123, 17, -456)
        self.assertEqual(uart.data, None)

        class Result:
            valid = 1
            lost_count = 2
            confidence = 876
            lateral_error_mm = -123
            heading_error_deg = 17
            curvature = -456

        self.assertTrue(sender.send(Result()))
        frame = uart.data
        self.assertEqual(len(frame), VISION_FRAME_LENGTH)
        self.assertEqual(frame[:6], bytes((0xAA, 0x55, 0x0D, 0x05, 0x04, 0x42)))
        self.assertEqual(frame[-2:], bytes((0x0D, 0x0A)))
        self.assertEqual(frame[6], 1)
        self.assertEqual(frame[7], 2)
        self.assertEqual(frame[8] | (frame[9] << 8), 876)
        self.assertEqual(read_i16_le(frame, 10), -123)
        self.assertEqual(read_i16_le(frame, 12), 17)
        self.assertEqual(read_i16_le(frame, 14), -456)

    def test_invalid_frame_has_no_stale_control_value(self):
        uart = FakeUart()
        sender = VisionLineSender(uart)
        self.assertTrue(sender.send_invalid(9))
        frame = uart.data
        self.assertEqual(frame[6], 0)
        self.assertEqual(frame[7], 9)
        self.assertEqual(frame[8:16], bytes(8))


if __name__ == "__main__":
    unittest.main()
