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
    def __init__(
        self, center_x=None, slope=0.0, width=16, width_by_strip=None
    ):
        self.center_x = center_x
        self.slope = slope
        self.width = width
        self.width_by_strip = width_by_strip
        self.call_index = 0
        self.draw_calls = 0
        self.binary_calls = 0
        self.dilate_calls = 0
        self.erode_calls = 0

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
        width = (
            self.width_by_strip[index]
            if self.width_by_strip is not None
            else self.width
        )
        return (FakeBlob(x, y, width=width),)

    def binary(self, _thresholds, **_kwargs):
        self.binary_calls += 1
        return self

    def dilate(self, _size):
        self.dilate_calls += 1
        return self

    def erode(self, _size):
        self.erode_calls += 1
        return self

    def draw_rectangle(self, *_args, **_kwargs):
        self.draw_calls += 1

    def draw_line(self, *_args, **_kwargs):
        self.draw_calls += 1

    def draw_cross(self, *_args, **_kwargs):
        self.draw_calls += 1

    def draw_string(self, *_args, **_kwargs):
        self.draw_calls += 1


class NoBinaryApiImage(FakeImage):
    def binary(self, _thresholds, **_kwargs):
        raise AttributeError("binary API unavailable")


class CompetingBlobImage(FakeImage):
    def find_blobs(self, _thresholds, **_kwargs):
        index = self.call_index % len(cfg.STRIP_CENTER_Y)
        self.call_index += 1
        y = cfg.STRIP_CENTER_Y[index]
        return (
            FakeBlob(cfg.OPTICAL_CENTER_X, y, width=16),
            FakeBlob(
                cfg.OPTICAL_CENTER_X + 8,
                y,
                width=cfg.MARKER_MIN_WIDTH + 12,
                height=3,
            ),
        )


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
        self.assertEqual(image.binary_calls, 2)
        self.assertEqual(
            image.dilate_calls, 2 * cfg.BINARY_DILATE_ITERATIONS
        )
        self.assertEqual(
            image.erode_calls, 2 * cfg.BINARY_ERODE_ITERATIONS
        )

    def test_line_to_right_has_positive_lateral_error(self):
        detector = LineDetector()
        image = FakeImage(center_x=cfg.OPTICAL_CENTER_X + 25)
        detector.process(image)
        result = detector.process(image)
        self.assertEqual(result.valid, 1)
        self.assertGreater(result.lateral_error_mm, 0)

    def test_binary_api_fallback_keeps_line_detection_running(self):
        detector = LineDetector()
        image = NoBinaryApiImage(center_x=cfg.OPTICAL_CENTER_X)
        detector.process(image)
        result = detector.process(image)
        self.assertEqual(result.valid, 1)
        self.assertEqual(result.lateral_error_mm, 0)

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

    def test_wide_track_marker_remains_drivable(self):
        detector = LineDetector()
        line = FakeImage(center_x=cfg.OPTICAL_CENTER_X)
        detector.process(line)
        self.assertEqual(detector.process(line).valid, 1)

        marker = FakeImage(
            center_x=cfg.OPTICAL_CENTER_X,
            width=cfg.MAX_LINE_WIDTH + 45,
        )
        first_marker = detector.process(marker)
        self.assertEqual(first_marker.marker_detected, 0)
        result = detector.process(marker)
        self.assertEqual(result.valid, 1)
        self.assertEqual(result.marker_detected, 1)
        self.assertGreaterEqual(result.marker_strips, cfg.MARKER_MIN_STRIPS)
        self.assertEqual(result.lateral_error_mm, 0)

        for _ in range(cfg.MARKER_OFF_CONFIRM_FRAMES - 1):
            recovered = detector.process(line)
            self.assertEqual(recovered.marker_detected, 1)
        recovered = detector.process(line)
        self.assertEqual(recovered.valid, 1)
        self.assertEqual(recovered.marker_detected, 0)

    def test_one_frame_wide_strip_is_not_a_marker(self):
        detector = LineDetector()
        line = FakeImage(center_x=cfg.OPTICAL_CENTER_X)
        detector.process(line)
        detector.process(line)

        widths = [16] * len(cfg.STRIP_CENTER_Y)
        widths[2] = cfg.MAX_LINE_WIDTH + 45
        noise = FakeImage(
            center_x=cfg.OPTICAL_CENTER_X,
            width_by_strip=widths,
        )
        result = detector.process(noise)
        self.assertEqual(result.marker_detected, 0)
        result = detector.process(line)
        self.assertEqual(result.marker_detected, 0)

    def test_marker_is_not_hidden_by_better_scoring_line_blob(self):
        detector = LineDetector()
        line = FakeImage(center_x=cfg.OPTICAL_CENTER_X)
        detector.process(line)
        detector.process(line)

        mixed = CompetingBlobImage(center_x=cfg.OPTICAL_CENTER_X)
        first = detector.process(mixed)
        self.assertEqual(first.marker_detected, 0)
        result = detector.process(mixed)
        self.assertEqual(result.marker_detected, 1)
        self.assertGreaterEqual(result.marker_strips, cfg.MARKER_MIN_STRIPS)
        self.assertEqual(result.valid, 1)

    def test_short_marker_dropout_does_not_split_one_dot(self):
        detector = LineDetector()
        line = FakeImage(center_x=cfg.OPTICAL_CENTER_X)
        marker = FakeImage(
            center_x=cfg.OPTICAL_CENTER_X,
            width=cfg.MAX_LINE_WIDTH + 45,
        )
        detector.process(line)
        detector.process(line)
        for _ in range(cfg.MARKER_ON_CONFIRM_FRAMES):
            result = detector.process(marker)
        self.assertEqual(result.marker_detected, 1)

        for _ in range(cfg.MARKER_OFF_CONFIRM_FRAMES - 1):
            result = detector.process(line)
            self.assertEqual(result.marker_detected, 1)

        result = detector.process(marker)
        self.assertEqual(result.marker_detected, 1)

    def test_wide_background_is_rejected_without_track_lock(self):
        detector = LineDetector()
        background = FakeImage(
            center_x=cfg.OPTICAL_CENTER_X,
            width=cfg.MAX_LINE_WIDTH + 45,
        )
        result = detector.process(background)
        self.assertEqual(result.valid, 0)
        self.assertEqual(result.marker_detected, 0)

    def test_debug_overlay_can_be_drawn(self):
        detector = LineDetector()
        image = FakeImage(center_x=cfg.OPTICAL_CENTER_X)
        detector.process(image)
        result = detector.process(image)
        self.assertTrue(detector.draw_debug_overlay(image, result, 55.0, 0))
        self.assertGreater(image.draw_calls, 0)


if __name__ == "__main__":
    unittest.main()
