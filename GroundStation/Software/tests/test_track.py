import math
import unittest

from ground_station.track import (
    TRACK_LENGTH_CM,
    drone_position_from_home,
    position_from_mileage,
)


class TrackTests(unittest.TestCase):
    def assert_position(self, mileage, x, y):
        actual_x, actual_y, _ = position_from_mileage(mileage)
        self.assertAlmostEqual(actual_x, x, places=5)
        self.assertAlmostEqual(actual_y, y, places=5)

    def test_key_points_a_b_c_d_a(self):
        self.assert_position(0, 150, 200)
        self.assert_position(150, 150, 350)
        self.assert_position(150 + 75 * math.pi, 300, 350)
        self.assert_position(300 + 75 * math.pi, 300, 200)
        self.assert_position(TRACK_LENGTH_CM, 150, 200)

    def test_cumulative_mileage_wraps_after_each_lap(self):
        first_lap = position_from_mileage(123)
        third_lap = position_from_mileage(2 * TRACK_LENGTH_CM + 123)
        for actual, expected in zip(third_lap, first_lap):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_aircraft_origin_is_map_point_h(self):
        self.assertEqual(drone_position_from_home(0, 0), (112.5, 112.5))

    def test_aircraft_axes_are_rotated_into_map_axes(self):
        field = drone_position_from_home(200, -40)
        self.assertEqual(field, (152.5, 312.5))


if __name__ == "__main__":
    unittest.main()
