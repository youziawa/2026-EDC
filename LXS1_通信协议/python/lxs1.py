"""Reference LXS1 parser/encoder for the ground station and K230."""

from __future__ import annotations

from dataclasses import dataclass
import struct

SOF = b"\xAA\x55"
EOF = b"\x0D\x0A"
MAX_DATA = 64


class ProtocolError(ValueError):
    pass


@dataclass(slots=True)
class Frame:
    src: int
    dst: int
    msg_id: int
    data: bytes = b""


def encode(frame: Frame) -> bytes:
    data = bytes(frame.data)
    if len(data) > MAX_DATA:
        raise ProtocolError("data exceeds 64 bytes")
    length = 3 + len(data)
    return SOF + bytes((length, frame.src, frame.dst, frame.msg_id)) + data + EOF


class Parser:
    """Incremental parser based on length and fixed frame tail."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames_ok = 0
        self.format_errors = 0

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                if self.buffer and self.buffer[-1] == SOF[0]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            length = self.buffer[2]
            if length < 3 or length > 3 + MAX_DATA:
                del self.buffer[0]
                self.format_errors += 1
                continue
            total = 2 + 1 + length + 2
            if len(self.buffer) < total:
                break
            if bytes(self.buffer[total - 2 : total]) != EOF:
                del self.buffer[0]
                self.format_errors += 1
                continue
            body = bytes(self.buffer[3 : 3 + length])
            del self.buffer[:total]
            frames.append(Frame(src=body[0], dst=body[1], msg_id=body[2], data=body[3:]))
            self.frames_ok += 1
        return frames


MSG_HELLO = 0x01
MSG_HEARTBEAT = 0x02
MSG_TASK_START = 0x10
MSG_TASK_ABORT = 0x11
MSG_TASK_STATE = 0x12
MSG_FC_CMD = 0x20
MSG_FC_STATE = 0x21
MSG_FC_POSE = 0x22
MSG_CAR_CMD = 0x30
MSG_CAR_STATE = 0x31
MSG_CAR_POSE = 0x32
MSG_TRACK_EVENT = 0x33
MSG_CAR_DIAGNOSTIC = 0x34
MSG_VISION_TARGET = 0x40
MSG_VISION_LANDMARK = 0x41
MSG_VISION_LINE = 0x42
MSG_VISION_DIAG = 0x43
MSG_DROP_STATE = 0x50
MSG_LAND_STATE = 0x51
MSG_FAULT = 0x60


def pack_task_start(
    run_id: int,
    task_mode: int,
    normal_speed_mm_s: int,
    action_speed_mm_s: int,
) -> bytes:
    return struct.pack(
        "<BBHH",
        run_id,
        task_mode,
        normal_speed_mm_s,
        action_speed_mm_s,
    )


def pack_task_state(
    run_id: int,
    task_mode: int,
    global_state: int,
    result: int,
    fault_code: int,
    elapsed_ms: int,
    car_path_mm: int,
) -> bytes:
    return struct.pack(
        "<BBBBHII",
        run_id,
        task_mode,
        global_state,
        result,
        fault_code,
        elapsed_ms,
        car_path_mm,
    )


def pack_vision_target(
    valid: int,
    target_kind: int,
    confidence_permille: int,
    dx_cm: int,
    dy_cm: int,
    dz_cm: int,
    yaw_error_deg: int,
) -> bytes:
    return struct.pack(
        "<BBHhhhh",
        1 if valid else 0,
        target_kind,
        confidence_permille,
        dx_cm if valid else 0,
        dy_cm if valid else 0,
        dz_cm if valid else 0,
        yaw_error_deg if valid else 0,
    )


def pack_vision_landmark(
    valid: int,
    marker_kind: int,
    confidence_permille: int,
    error_x_cm: int,
    error_y_cm: int,
    error_yaw_deg: int,
) -> bytes:
    return struct.pack(
        "<BBHhhh",
        1 if valid else 0,
        marker_kind,
        confidence_permille,
        error_x_cm if valid else 0,
        error_y_cm if valid else 0,
        error_yaw_deg if valid else 0,
    )


def pack_vision_line(
    valid: int,
    lost_count: int,
    confidence_permille: int,
    lateral_error_mm: int,
    heading_error_deg: int,
    curvature: int,
    marker_detected: int = 0,
) -> bytes:
    return struct.pack(
        "<BBHhhhB",
        valid,
        lost_count,
        confidence_permille,
        lateral_error_mm,
        heading_error_deg,
        curvature,
        1 if marker_detected else 0,
    )
