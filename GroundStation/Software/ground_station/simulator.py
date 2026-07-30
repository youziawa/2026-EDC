from __future__ import annotations

from queue import Queue
import struct
import threading
import time
from typing import Any

from .protocol import Frame, encode
from .track import TRACK_LENGTH_CM, position_from_mileage

LAP_DURATION_S = 60.0
POINT_C_MILEAGE_CM = 150.0 + 75.0 * 3.141592653589793
POINT_D_MILEAGE_CM = 300.0 + 75.0 * 3.141592653589793
POINT_C_TIME_S = LAP_DURATION_S * POINT_C_MILEAGE_CM / TRACK_LENGTH_CM
POINT_D_TIME_S = LAP_DURATION_S * POINT_D_MILEAGE_CM / TRACK_LENGTH_CM
DROP_START_S = 31.0
DROP_DONE_S = 34.0
LAND_CAR_START_S = 30.5
LAND_CAR_DONE_S = 35.0
LAND_CAR_WAIT_DONE_S = 40.0


def _stadium_position(progress: float) -> tuple[float, float, float]:
    """Clockwise A→B→C→D→A track matching the problem map."""
    return position_from_mileage((progress % 1.0) * TRACK_LENGTH_CM)


class Simulator:
    def __init__(self, events: Queue[tuple[str, Any]]) -> None:
        self.events = events
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.started_at = 0.0

    @property
    def running(self) -> bool:
        return bool(self.thread and self.thread.is_alive())

    def start(self, task_mode: int = 2) -> None:
        self.stop()
        self.stop_event.clear()
        self.started_at = time.monotonic()
        self.thread = threading.Thread(
            target=self._loop,
            args=(task_mode,),
            name="telemetry-demo",
            daemon=True,
        )
        self.thread.start()
        mode_name = "抛投任务" if task_mode == 1 else "动态起降"
        self.events.put(("status", f"{mode_name}演示运行中"))

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=0.6)
        self.thread = None

    def _emit(self, frame: Frame) -> None:
        packet = encode(frame)
        split = 3 + (int(time.monotonic() * 10) % max(1, len(packet) - 3))
        self.events.put(("bytes", packet[:split]))
        self.events.put(("bytes", packet[split:]))

    def _loop(self, task_mode: int) -> None:
        self._emit(Frame(0x02, 0x06, 0x01, b"DRONE"))
        self._emit(Frame(0x04, 0x06, 0x01, b"CAR"))
        self._emit(
            Frame(
                0x04,
                0x06,
                0x10,
                struct.pack("<BBHH", 1, task_mode, 150, 80),
            )
        )
        while not self.stop_event.wait(0.1):
            elapsed = (time.monotonic() - self.started_at) % 65.0
            progress = min(elapsed / LAP_DURATION_S, 1.0)
            x, y, heading = _stadium_position(progress)
            if elapsed < 3:
                task_state, z = 1, elapsed / 3 * 150
            elif elapsed < 6:
                task_state, z = 2, 150
            elif elapsed < 10:
                task_state, z = 3, 150
            elif task_mode == 1 and elapsed < DROP_START_S:
                task_state, z = 4, 150
            elif task_mode == 1 and elapsed < DROP_DONE_S:
                task_state, z = 5, 150
            elif task_mode == 2 and elapsed < LAND_CAR_START_S:
                task_state, z = 4, 150
            elif task_mode == 2 and elapsed < LAND_CAR_DONE_S:
                task_state, z = 6, max(12, 150 - (elapsed - LAND_CAR_START_S) * 31)
            elif task_mode == 2 and elapsed < LAND_CAR_WAIT_DONE_S:
                task_state, z = 7, 12
            elif task_mode == 2 and elapsed < 42:
                task_state, z = 8, 12 + (elapsed - LAND_CAR_WAIT_DONE_S) * 69
            elif elapsed < (46 if task_mode == 1 else 49):
                task_state, z = 9, 150
            elif elapsed < (53 if task_mode == 1 else 56):
                land_start = 46 if task_mode == 1 else 49
                task_state, z = 10, max(0, 150 - (elapsed - land_start) * 22)
            else:
                task_state, z = 11, 0

            home_x, home_y = 112.5, 112.5
            if task_state in (3, 4, 5, 6, 7, 8):
                drone_x, drone_y = x, y
            elif task_state == 9:
                return_start = DROP_DONE_S if task_mode == 1 else 42
                return_duration = (46 if task_mode == 1 else 49) - return_start
                start_x, start_y, _ = _stadium_position(return_start / LAP_DURATION_S)
                ratio = min(1.0, (elapsed - return_start) / return_duration)
                drone_x = start_x + (home_x - start_x) * ratio
                drone_y = start_y + (home_y - start_y) * ratio
            else:
                drone_x, drone_y = home_x, home_y

            path_mm = int(progress * TRACK_LENGTH_CM * 10)
            path_cm = path_mm // 10
            self._emit(Frame(0x04, 0x06, 0x32, struct.pack("<HHhH", int(x), int(y), int(heading), path_cm)))
            car_state = 2 if elapsed < LAP_DURATION_S else 7
            car_state_data = struct.pack(
                "<BBBBHHIB",
                1,                         # run_id
                car_state,
                1 if elapsed < LAP_DURATION_S else 0, # speed_mode
                1,                         # line_valid
                130 if elapsed < LAP_DURATION_S else 0,
                0,                         # fault
                path_mm,
                int(elapsed >= LAP_DURATION_S),
            )
            self._emit(Frame(0x04, 0x06, 0x31, car_state_data))
            self._emit(Frame(0x01, 0x06, 0x22, struct.pack("<hhhh", int(drone_x), int(drone_y), int(z), int(heading))))
            fc_state = {
                0: 0, 1: 1, 2: 2, 3: 3, 4: 3, 5: 2,
                6: 4, 7: 0, 8: 1, 9: 5, 10: 6, 11: 0,
            }.get(task_state, 0)
            self._emit(Frame(0x01, 0x06, 0x21, struct.pack("<BBHhhhHB", fc_state, int(z > 0), int(z), 0, 0, 0, 15900, 0)))
            task = struct.pack(
                "<BBBBHII",
                1,            # run_id
                task_mode,
                task_state,
                0,            # result
                0,            # fault_code
                int(elapsed * 1000),
                path_mm,
            )
            self._emit(Frame(0x02, 0x06, 0x12, task))
            if task_mode == 1:
                drop_state = 0 if elapsed < DROP_START_S else 1 if elapsed < 32 else 2
                self._emit(Frame(0x02, 0x06, 0x50, bytes((drop_state, 25))))
            elif elapsed < LAND_CAR_START_S:
                self._emit(Frame(0x02, 0x06, 0x51, b"\0\0\0"))
            elif elapsed < LAND_CAR_DONE_S:
                self._emit(Frame(0x02, 0x06, 0x51, bytes((1,)) + b"\0\0"))
            elif elapsed < LAND_CAR_WAIT_DONE_S:
                self._emit(Frame(0x02, 0x06, 0x51, bytes((3,)) + int(elapsed - LAND_CAR_DONE_S).to_bytes(2, "little")))
            else:
                self._emit(Frame(0x02, 0x06, 0x51, bytes((4, 5, 0))))
