import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "python"))

from lxs1 import (
    Frame,
    Parser,
    encode,
    pack_task_start,
    pack_task_state,
    pack_vision_line,
    pack_vision_pixel,
)
from car_diagnostic_monitor import PAYLOAD, decode_diagnostic, diagnoses


class LXS1Tests(unittest.TestCase):
    def test_roundtrip(self):
        expected = Frame(
            src=0x03,
            dst=0x02,
            msg_id=0x40,
            data=pack_vision_line(1, 0, 950, -12, 4, 0),
        )
        actual = Parser().feed(encode(expected))[0]
        self.assertEqual(actual, expected)

    def test_fragmentation_and_coalescing(self):
        first = encode(Frame(src=3, dst=2, msg_id=1, data=b"hello"))
        second = encode(Frame(src=2, dst=6, msg_id=2, data=b"world"))
        parser = Parser()
        output = []
        stream = first + second
        for offset in range(0, len(stream), 3):
            output.extend(parser.feed(stream[offset : offset + 3]))
        self.assertEqual(
            output,
            [
                Frame(src=3, dst=2, msg_id=1, data=b"hello"),
                Frame(src=2, dst=6, msg_id=2, data=b"world"),
            ],
        )

    def test_bad_tail_resynchronizes(self):
        damaged = bytearray(encode(Frame(src=3, dst=2, msg_id=1, data=b"bad")))
        damaged[-1] = 0x00
        valid = encode(Frame(src=3, dst=2, msg_id=2, data=b"good"))
        parser = Parser()
        self.assertEqual(
            parser.feed(damaged + valid),
            [Frame(src=3, dst=2, msg_id=2, data=b"good")],
        )
        self.assertGreaterEqual(parser.format_errors, 1)

    def test_vision_line_includes_marker_flag(self):
        data = pack_vision_line(1, 0, 900, -3, 2, 10, 1)
        self.assertEqual(len(data), 11)
        self.assertEqual(data[-1], 1)

    def test_current_task_payload_layouts(self):
        self.assertEqual(
            pack_task_start(7, 1, 150, 80),
            bytes((7, 1, 150, 0, 80, 0)),
        )
        task_state = pack_task_state(7, 1, 9, 0, 0, 1234, 5356)
        self.assertEqual(len(task_state), 14)
        self.assertEqual(task_state[:4], bytes((7, 1, 9, 0)))

    def test_air_vision_payload_layouts_and_invalid_zeroing(self):
        pixel = pack_vision_pixel(1, 2, 900, 10, -20, 68, 2)
        self.assertEqual(len(pixel), 12)
        invalid_pixel = pack_vision_pixel(0, 2, 0, 10, 20, 68, 3)
        self.assertEqual(invalid_pixel[4:10], bytes(6))
        self.assertEqual(invalid_pixel[10:12], bytes((3, 0)))


    def test_car_diagnostic_points_to_driver_wiring(self):
        data = PAYLOAD.pack(
            1, 1, 0, 0x63, 0,
            11200, 11200, 1000, 1000, 0, 0,
            120, 120, 0, 0, 10, 20, 0, 0,
        )
        diagnostic = decode_diagnostic(data)
        suggestions = "\n".join(diagnoses(diagnostic))
        self.assertIn("软件已输出PWM且STBY为高", suggestions)

    def test_car_diagnostic_reports_start_countdown(self):
        data = PAYLOAD.pack(
            0, 0, 0, 0x80, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0xFFFF, 0, 0, 0,
        )
        suggestions = "\n".join(diagnoses(decode_diagnostic(data)))
        self.assertIn("10 s安全撤离倒计时", suggestions)
        self.assertNotIn("开始键未到达PC0", suggestions)


if __name__ == "__main__":
    unittest.main()
