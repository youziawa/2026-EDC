import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "python"))

from lxs1 import Frame, Parser, encode, pack_vision_line


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


if __name__ == "__main__":
    unittest.main()
