from __future__ import annotations

from dataclasses import dataclass

SOF = b"\xAA\x55"
EOF = b"\x0D\x0A"
MAX_DATA = 64

GROUND_STATION = 0x06
BROADCAST = 0xFF

MSG_NAMES = {
    0x01: "HELLO",
    0x02: "HEARTBEAT",
    0x10: "TASK_START",
    0x11: "TASK_ABORT",
    0x12: "TASK_STATE",
    0x20: "FC_CMD",
    0x21: "FC_STATE",
    0x22: "FC_POSE",
    0x30: "CAR_CMD",
    0x31: "CAR_STATE",
    0x32: "CAR_POSE",
    0x33: "TRACK_EVENT",
    0x34: "CAR_DIAGNOSTIC",
    0x40: "VISION_TARGET",
    0x41: "VISION_LANDMARK",
    0x42: "VISION_LINE",
    0x43: "VISION_DIAG",
    0x50: "DROP_STATE",
    0x51: "LAND_STATE",
    0x60: "FAULT",
}

DEVICE_NAMES = {
    0x01: "飞控适配端",
    0x02: "飞机主控",
    0x03: "天空视觉",
    0x04: "小车主控",
    0x05: "小车视觉",
    0x06: "地面站",
    0xFF: "广播",
}


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class Frame:
    src: int
    dst: int
    msg_id: int
    data: bytes = b""


def encode(frame: Frame) -> bytes:
    data = bytes(frame.data)
    if not all(0 <= value <= 0xFF for value in (frame.src, frame.dst, frame.msg_id)):
        raise ProtocolError("address and message fields must be bytes")
    if len(data) > MAX_DATA:
        raise ProtocolError("data exceeds 64 bytes")
    return SOF + bytes((3 + len(data), frame.src, frame.dst, frame.msg_id)) + data + EOF


class Parser:
    """Length-based incremental parser with resynchronisation."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames_ok = 0
        self.format_errors = 0

    def feed(self, incoming: bytes | bytearray | memoryview) -> list[Frame]:
        self.buffer.extend(incoming)
        frames: list[Frame] = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == SOF[:1] else b""
                return frames
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 3:
                return frames
            length = self.buffer[2]
            if length < 3 or length > 3 + MAX_DATA:
                del self.buffer[0]
                self.format_errors += 1
                continue
            total = 2 + 1 + length + 2
            if len(self.buffer) < total:
                return frames
            if self.buffer[total - 2 : total] != EOF:
                del self.buffer[0]
                self.format_errors += 1
                continue
            body = bytes(self.buffer[3 : 3 + length])
            del self.buffer[:total]
            frames.append(Frame(body[0], body[1], body[2], body[3:]))
            self.frames_ok += 1
