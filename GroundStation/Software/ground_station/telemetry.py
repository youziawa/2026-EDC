from __future__ import annotations

from dataclasses import dataclass, field
import struct
import time

from .protocol import DEVICE_NAMES, Frame, MSG_NAMES
from .track import position_from_mileage

TASK_STATES = [
    "待机", "起飞", "悬停 3 秒", "搜索小车", "伴飞", "抛投",
    "降落小车", "平台停留 5 秒", "再次起飞", "返航", "降落 H 点",
    "完成", "已终止", "故障",
]
FC_STATES = ["待机", "起飞", "悬停", "跟踪", "下降", "返航", "降落", "终止"]
CAR_MISSION_STATES = [
    "待机", "起步寻线", "正常循线", "动作区慢行", "丢线保持",
    "重新捕线", "终点确认", "完成", "终止", "故障",
]
LEGACY_CAR_STATES = ["待机", "运行", "完成", "故障", "终止"]
DROP_STATES = ["未动作", "释放", "完成", "失败"]
LAND_STATES = ["未开始", "下降", "已着陆", "停留", "完成", "失败"]
TASK_MODE_TITLES = {
    1: "题目一（抛投）",
    2: "题目二（动态起降）",
}


def _name(items: list[str], value: int) -> str:
    return items[value] if 0 <= value < len(items) else f"未知({value})"


def task_mode_title(task_mode: int) -> str:
    return TASK_MODE_TITLES.get(task_mode, "题目未选择")


def task_execution_label(task_mode: int, task_state: str) -> str:
    title = TASK_MODE_TITLES.get(task_mode)
    if title is None:
        return f"等待任务 · {task_state}"
    if task_state == "完成":
        prefix = "已完成"
    elif task_state == "已终止":
        prefix = "已终止"
    elif task_state == "故障":
        prefix = "故障"
    else:
        prefix = "正在执行"
    return f"{prefix}：{title} · {task_state}"


def _u24(data: bytes) -> int:
    return int.from_bytes(data[:3], "little", signed=False)


@dataclass(slots=True)
class Vehicle:
    x: float | None = None
    y: float | None = None
    z: float | None = None
    heading: float = 0.0
    speed: float = 0.0
    path: float = 0.0
    state: str = "未知"
    fault: int = 0
    last_update: float = 0.0


@dataclass(slots=True)
class Telemetry:
    car: Vehicle = field(default_factory=Vehicle)
    drone: Vehicle = field(default_factory=Vehicle)
    task_mode: int = 0
    task_state: str = "待机"
    task_result: int = 0
    elapsed_ms: int = 0
    battery_mv: int = 0
    armed: bool = False
    lap: int = 0
    line_valid: bool = False
    drop_state: str = "未动作"
    land_state: str = "未开始"
    stable_time_s: int = 0
    run_id: int = 0
    last_frame_at: float = 0.0
    last_direct_car_state_at: float = 0.0
    frames_received: int = 0
    last_by_source: dict[int, float] = field(default_factory=dict)

    def _set_car_mileage(self, path_cm: float, now: float) -> None:
        self.car.path = path_cm
        self.car.x, self.car.y, self.car.heading = position_from_mileage(path_cm)
        self.car.last_update = now

    def apply(self, frame: Frame, now: float | None = None) -> list[str]:
        now = time.monotonic() if now is None else now
        data = frame.data
        events: list[str] = []
        self.frames_received += 1
        self.last_frame_at = now
        self.last_by_source[frame.src] = now

        try:
            if frame.msg_id == 0x01:
                label = data.rstrip(b"\0").decode("utf-8", "replace") or "设备"
                events.append(f"{DEVICE_NAMES.get(frame.src, hex(frame.src))} 上线：{label}")
            elif frame.msg_id == 0x10 and len(data) >= 6:
                self.run_id = data[0]
                self.task_mode = data[1]
                events.append(
                    f"任务启动：run_id={self.run_id}，"
                    f"模式={task_mode_title(self.task_mode)}"
                )
            elif frame.msg_id == 0x12 and len(data) >= 14:
                # LXS1 v1.1: run_id, task_mode, global_state, result,
                # fault_code(u16), elapsed_ms(u32), car_path_mm(u32).
                old = self.task_state
                self.run_id = data[0]
                self.task_mode = data[1]
                self.task_state = _name(TASK_STATES, data[2])
                self.task_result = data[3]
                self.drone.fault = int.from_bytes(data[4:6], "little")
                self.elapsed_ms = int.from_bytes(data[6:10], "little")
                # Direct CAR_STATE odometry is authoritative. TASK_STATE may
                # contain a forwarded/stale copy, so only use it as fallback.
                if now - self.last_direct_car_state_at > 1.0:
                    self._set_car_mileage(
                        int.from_bytes(data[10:14], "little") / 10.0, now
                    )
                if old != self.task_state:
                    events.append(f"任务状态：{old} → {self.task_state}")
            elif frame.msg_id == 0x12 and len(data) >= 9:
                # Compatibility with the original 9-byte draft protocol.
                old = self.task_state
                self.task_mode = data[0]
                self.task_state = _name(TASK_STATES, data[1])
                self.drone.fault = data[2]
                self.elapsed_ms = _u24(data[3:6])
                if now - self.last_direct_car_state_at > 1.0:
                    self._set_car_mileage(_u24(data[6:9]) / 10.0, now)
                if old != self.task_state:
                    events.append(f"任务状态：{old} → {self.task_state}")
            elif frame.msg_id == 0x21 and len(data) >= 13:
                values = struct.unpack_from("<BBHhhhHB", data)
                self.drone.state = _name(FC_STATES, values[0])
                self.armed = bool(values[1])
                self.drone.z = values[2]
                self.drone.speed = (values[3] ** 2 + values[4] ** 2) ** 0.5
                self.battery_mv = values[6]
                self.drone.fault = values[7]
                self.drone.last_update = now
            elif frame.msg_id == 0x22 and len(data) >= 4:
                self.drone.x, self.drone.y = struct.unpack_from("<hh", data)
                if len(data) >= 6:
                    self.drone.z = struct.unpack_from("<h", data, 4)[0]
                if len(data) >= 8:
                    self.drone.heading = struct.unpack_from("<h", data, 6)[0]
                self.drone.last_update = now
            elif frame.msg_id == 0x31 and len(data) >= 13:
                # Current CarF407 firmware payload:
                # run_id, state, speed_mode, line_valid, speed_mm_s(u16),
                # fault(u16), path_mm(u32), lap_complete.
                self.run_id = data[0]
                self.car.state = _name(CAR_MISSION_STATES, data[1])
                self.line_valid = bool(data[3])
                self.car.speed = int.from_bytes(data[4:6], "little") / 10.0
                self.car.fault = int.from_bytes(data[6:8], "little")
                self._set_car_mileage(
                    int.from_bytes(data[8:12], "little") / 10.0, now
                )
                self.lap = 1 if data[12] else 0
                self.last_direct_car_state_at = now
            elif frame.msg_id == 0x31 and len(data) >= 7:
                # Compatibility with the original 7-byte protocol document.
                self.car.state = _name(LEGACY_CAR_STATES, data[0])
                self.car.speed = data[1]
                self._set_car_mileage(int.from_bytes(data[2:4], "little"), now)
                self.lap = data[4]
                self.line_valid = bool(data[5])
                self.car.fault = data[6]
                self.last_direct_car_state_at = now
            elif frame.msg_id == 0x32 and len(data) >= 8:
                # The field display intentionally follows odometry mileage.
                # CAR_POSE can be inaccurate when the car only performs line
                # following, so it must not overwrite the derived map pose.
                self.car.last_update = now
            elif frame.msg_id == 0x50 and len(data) >= 1:
                old = self.drop_state
                self.drop_state = _name(DROP_STATES, data[0])
                if old != self.drop_state:
                    events.append(f"抛投状态：{old} → {self.drop_state}")
            elif frame.msg_id == 0x51 and len(data) >= 1:
                old = self.land_state
                self.land_state = _name(LAND_STATES, data[0])
                if len(data) >= 3:
                    self.stable_time_s = int.from_bytes(data[1:3], "little")
                if old != self.land_state:
                    events.append(f"降落状态：{old} → {self.land_state}")
            elif frame.msg_id == 0x60 and len(data) >= 3:
                severity = {1: "提示", 2: "警告", 3: "错误"}.get(
                    data[2], f"级别{data[2]}"
                )
                events.append(f"{severity}：源 0x{data[0]:02X}，故障码 0x{data[1]:02X}")
        except (struct.error, IndexError) as exc:
            events.append(f"{MSG_NAMES.get(frame.msg_id, hex(frame.msg_id))} 数据长度错误：{exc}")
        return events
