import math
import unittest

from ground_station.simulator import (
    DROP_DONE_S,
    DROP_START_S,
    LAND_CAR_START_S,
    LAND_CAR_WAIT_DONE_S,
    LAP_DURATION_S,
    POINT_C_MILEAGE_CM,
    POINT_D_MILEAGE_CM,
    _stadium_position,
)
from ground_station.track import TRACK_LENGTH_CM


class SimulatorTests(unittest.TestCase):
    def test_track_stays_inside_field(self):
        for step in range(1001):
            x, y, _heading = _stadium_position(step / 1000)
            self.assertGreaterEqual(x, 0)
            self.assertLessEqual(x, 400)
            self.assertGreaterEqual(y, 0)
            self.assertLessEqual(y, 500)

    def test_track_is_continuous(self):
        last = _stadium_position(0)
        for step in range(1, 1000):
            current = _stadium_position(step / 1000)
            self.assertLess(math.hypot(current[0] - last[0], current[1] - last[1]), 2)
            last = current

    def test_clockwise_order_is_a_b_c_d_a(self):
        total = 300 + 150 * math.pi
        a = _stadium_position(0)
        just_after_a = _stadium_position(1 / total)
        b = _stadium_position(150 / total)
        c = _stadium_position((150 + 75 * math.pi) / total)
        d = _stadium_position((300 + 75 * math.pi) / total)
        self.assertEqual(a[:2], (150, 200))
        self.assertGreater(just_after_a[1], a[1], "A 点后必须向 B 点向上行驶")
        self.assertAlmostEqual(b[0], 150, places=5)
        self.assertAlmostEqual(b[1], 350, places=5)
        self.assertAlmostEqual(c[0], 300, places=5)
        self.assertAlmostEqual(c[1], 350, places=5)
        self.assertAlmostEqual(d[0], 300, places=5)
        self.assertAlmostEqual(d[1], 200, places=5)

    def test_drop_and_car_landing_are_inside_c_d_segment(self):
        def mileage_at(seconds):
            return seconds / LAP_DURATION_S * TRACK_LENGTH_CM

        for action_time in (
            DROP_START_S,
            DROP_DONE_S,
            LAND_CAR_START_S,
            LAND_CAR_WAIT_DONE_S,
        ):
            mileage = mileage_at(action_time)
            self.assertGreaterEqual(mileage, POINT_C_MILEAGE_CM)
            self.assertLessEqual(mileage, POINT_D_MILEAGE_CM)


if __name__ == "__main__":
    unittest.main()
