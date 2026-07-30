import unittest

from ground_station.protocol import Frame, Parser, ProtocolError, encode


class ProtocolTests(unittest.TestCase):
    def test_roundtrip(self):
        frame = Frame(3, 2, 0x40, b"\x01\x02hello")
        self.assertEqual(Parser().feed(encode(frame)), [frame])

    def test_fragmentation_and_multiple_frames(self):
        expected = [Frame(2, 6, 1, b"a"), Frame(4, 6, 2, b"b")]
        stream = b"noise" + b"".join(map(encode, expected))
        parser = Parser()
        actual = []
        for index in range(0, len(stream), 2):
            actual.extend(parser.feed(stream[index:index + 2]))
        self.assertEqual(actual, expected)

    def test_bad_tail_resynchronises(self):
        bad = bytearray(encode(Frame(1, 6, 2, b"bad")))
        bad[-1] = 0
        good = Frame(4, 6, 2, b"good")
        parser = Parser()
        self.assertEqual(parser.feed(bad + encode(good)), [good])
        self.assertGreater(parser.format_errors, 0)

    def test_rejects_large_payload(self):
        with self.assertRaises(ProtocolError):
            encode(Frame(1, 2, 3, bytes(65)))


if __name__ == "__main__":
    unittest.main()
