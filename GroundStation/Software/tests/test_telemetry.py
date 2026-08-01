import struct
import unittest

from ground_station.protocol import Frame
from ground_station.telemetry import Telemetry, task_execution_label, task_mode_title


class TelemetryTests(unittest.TestCase):
    def test_task_mode_is_displayed_as_question_number(self):
        self.assertEqual(task_mode_title(1), "题目一（抛投）")
        self.assertEqual(task_mode_title(2), "题目二（动态起降）")
        self.assertEqual(
            task_execution_label(2, "降落小车"),
            "正在执行：题目二（动态起降） · 降落小车",
        )

    def test_car_position_is_derived_from_mileage(self):
        model = Telemetry()
        model.apply(Frame(4, 6, 0x31, struct.pack("<BBHBBB", 1, 20, 100, 0, 1, 0)), 1.0)
        self.assertEqual((model.car.x, model.car.y, model.car.heading), (150, 300, 90))
        self.assertEqual(model.car.state, "运行")
        self.assertTrue(model.line_valid)

    def test_car_pose_does_not_override_odometry_position(self):
        model = Telemetry()
        model.apply(Frame(4, 6, 0x31, struct.pack("<BBHBBB", 1, 20, 100, 0, 1, 0)), 1.0)
        model.apply(Frame(4, 6, 0x32, struct.pack("<HHhH", 399, 499, -90, 999)), 2.0)
        self.assertEqual((model.car.x, model.car.y, model.car.path), (150, 300, 100))

    def test_current_firmware_car_state_layout(self):
        model = Telemetry()
        # This is the exact 13-byte layout emitted by CarF407 car_mission.c.
        data = (
            bytes((7, 2, 1, 1))
            + (150).to_bytes(2, "little")
            + (0).to_bytes(2, "little")
            + (1000).to_bytes(4, "little")
            + bytes((0,))
        )
        model.apply(Frame(4, 6, 0x31, data), 10.0)
        self.assertEqual(model.run_id, 7)
        self.assertEqual(model.car.state, "正常循线")
        self.assertTrue(model.line_valid)
        self.assertEqual(model.car.speed, 15)
        self.assertEqual(model.car.path, 100)
        self.assertEqual((model.car.x, model.car.y), (150, 300))

    def test_direct_car_state_wins_over_stale_task_state_path(self):
        model = Telemetry()
        state = (
            bytes((1, 2, 1, 1))
            + (150).to_bytes(2, "little")
            + b"\0\0"
            + (1000).to_bytes(4, "little")
            + b"\0"
        )
        model.apply(Frame(4, 6, 0x31, state), 10.0)
        stale_task = bytes((2, 4, 0)) + (1000).to_bytes(3, "little") + (2680).to_bytes(3, "little")
        model.apply(Frame(2, 6, 0x12, stale_task), 10.1)
        self.assertEqual(model.car.path, 100)
        self.assertEqual((model.car.x, model.car.y), (150, 300))

    def test_task_state_u24(self):
        model = Telemetry()
        data = bytes((2, 4, 0)) + (12345).to_bytes(3, "little") + (54321).to_bytes(3, "little")
        events = model.apply(Frame(2, 6, 0x12, data), 1.0)
        self.assertEqual(model.task_state, "伴飞")
        self.assertEqual(model.elapsed_ms, 12345)
        self.assertTrue(events)

    def test_lxs1_v11_task_state(self):
        model = Telemetry()
        data = struct.pack("<BBBBHII", 9, 2, 6, 0, 0, 32100, 4200)
        events = model.apply(Frame(2, 6, 0x12, data), 5.0)
        self.assertEqual(model.run_id, 9)
        self.assertEqual(model.task_mode, 2)
        self.assertEqual(model.task_state, "降落小车")
        self.assertEqual(model.elapsed_ms, 32100)
        self.assertEqual(model.car.path, 420)
        self.assertTrue(events)

    def test_fc_state(self):
        model = Telemetry()
        data = struct.pack("<BBHhhhHB", 3, 1, 150, 30, 40, 0, 16000, 0)
        model.apply(Frame(1, 6, 0x21, data), 1.0)
        self.assertEqual(model.drone.state, "跟踪")
        self.assertEqual(model.drone.speed, 50)
        self.assertEqual(model.battery_mv, 16000)

    def test_air_f407_xy_pose_updates_drone_position(self):
        model = Telemetry()
        data = struct.pack("<hh", 200, -40)
        model.apply(Frame(2, 6, 0x22, data), 2.5)
        self.assertEqual((model.drone.x, model.drone.y), (200, -40))
        self.assertEqual(model.drone.last_update, 2.5)


if __name__ == "__main__":
    unittest.main()
