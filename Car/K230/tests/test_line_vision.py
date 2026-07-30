import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import vision_config as cfg
from line_vision import LineDetector


class FakeThreshold:
    def value(self):
        return 84


class FakeHistogram:
    def get_threshold(self):
        return FakeThreshold()


class FakeBlob:
    def __init__(self, center_x, center_y, width=16, height=18):
        self._cx = center_x
        self._cy = center_y
        self._w = width
        self._h = height

    def cx(self):
        return self._cx

    def cy(self):
        return self._cy

    def w(self):
        return self._w

    def h(self):
        return self._h

    def pixels(self):
        return self._w * self._h

    def rect(self):
        return (
            self._cx - self._w // 2,
            self._cy - self._h // 2,
            self._w,
            self._h,
        )


class FakeImage:
    def __init__(self, center_x=None, slope=0.0):
        self.center_x = center_x
        self.slope = slope
        self.call_index = 0
        self.draw_calls = 0

    def get_histogram(self, **_kwargs):
        return FakeHistogram()

    def find_blobs(self, _thresholds, **_kwargs):
        index = self.call_index % len(cfg.STRIP_CENTER_Y)
        self.call_index += 1
        if self.center_x is None:
            return ()

        y = cfg.STRIP_CENTER_Y[index]
        forward = cfg.IMAGE_HEIGHT - y
        x = int(round(self.center_x + self.slope * forward))
        return (FakeBlob(x, y),)

    def draw_rectangle(self, *_args, **_kwargs):
        self.draw_calls += 1

    def draw_line(self, *_args, **_kwargs):
        self.draw_calls += 1

    def draw_cross(self, *_args, **_kwargs):
        self.draw_calls += 1

    def draw_string(self, *_args, **_kwargs):
        self.draw_calls += 1


class LineDetectorTest(unittest.TestCase):
    def test_center_line_becomes_valid_after_confirmation(self):
        detector = LineDetector()
        image = FakeImage(center_x=cfg.OPTICAL_CENTER_X)
        first = detector.process(image)
        self.assertEqual(first.valid, 0)
        second = detector.process(image)
        self.assertEqual(second.valid, 1)
        self.assertGreaterEqual(second.confidence, cfg.MIN_CONFIDENCE)
        self.assertEqual(second.lateral_error_mm, 0)
        self.assertEqual(second.heading_error_deg, 0)

    def test_line_to_right_has_positive_lateral_error(self):
        detector = LineDetector()
        image = FakeImage(center_x=cfg.OPTICAL_CENTER_X + 25)
        detector.process(image)
        result = detector.process(image)
        self.assertEqual(result.valid, 1)
        self.assertGreater(result.lateral_error_mm, 0)

    def test_loss_is_immediate_and_clears_values(self):
        detector = LineDetector()
        line = FakeImage(center_x=cfg.OPTICAL_CENTER_X + 20, slope=0.10)
        detector.process(line)
        valid = detector.process(line)
        self.assertEqual(valid.valid, 1)

        lost = detector.process(FakeImage(center_x=None))
        self.assertEqual(lost.valid, 0)
        self.assertEqual(lost.lost_count, 1)
        self.assertEqual(lost.confidence, 0)
        self.assertEqual(lost.lateral_error_mm, 0)
        self.assertEqual(lost.heading_error_deg, 0)
        self.assertEqual(lost.curvature, 0)

    def test_debug_overlay_can_be_drawn(self):
        detector = LineDetector()
        image = FakeImage(center_x=cfg.OPTICAL_CENTER_X)
        detector.process(image)
        result = detector.process(image)
        self.assertTrue(detector.draw_debug_overlay(image, result, 55.0, 0))
        self.assertGreater(image.draw_calls, 0)


if __name__ == "__main__":
    unittest.main()
