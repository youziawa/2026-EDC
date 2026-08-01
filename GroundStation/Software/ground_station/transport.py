from __future__ import annotations

from queue import Queue
import threading
import time
from typing import Any


def serial_available() -> bool:
    try:
        import serial  # noqa: F401
        return True
    except ImportError:
        return False


def list_ports() -> list[tuple[str, str]]:
    try:
        from serial.tools import list_ports as serial_ports
    except ImportError:
        return []
    return [(port.device, port.description) for port in serial_ports.comports()]


class SerialTransport:
    def __init__(
        self,
        events: Queue[tuple[str, str, Any]],
        channel: str,
        display_name: str,
    ) -> None:
        self.events = events
        self.channel = channel
        self.display_name = display_name
        self.serial = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()

    @property
    def connected(self) -> bool:
        return bool(self.serial and self.serial.is_open)

    @property
    def port(self) -> str | None:
        return self.serial.port if self.serial else None

    def connect(self, port: str, baudrate: int) -> None:
        import serial

        self.disconnect()
        self.serial = serial.Serial(port, baudrate, timeout=0.1, write_timeout=0.5)
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._read_loop,
            name=f"{self.channel}-serial-reader",
            daemon=True,
        )
        self.thread.start()
        self.events.put((
            "status",
            self.channel,
            f"{self.display_name}已连接 {port} @ {baudrate}",
        ))

    def _read_loop(self) -> None:
        while not self.stop_event.is_set() and self.serial:
            try:
                data = self.serial.read(max(1, self.serial.in_waiting))
                if data:
                    self.events.put(("bytes", self.channel, data))
            except Exception as exc:
                self.events.put((
                    "error",
                    self.channel,
                    f"{self.display_name}串口读取失败：{exc}",
                ))
                break
        if not self.stop_event.is_set():
            current = self.serial
            self.serial = None
            if current:
                try:
                    current.close()
                except Exception:
                    pass
            self.events.put(("disconnected", self.channel, None))

    def write(self, data: bytes) -> None:
        if not self.connected:
            raise RuntimeError("串口未连接")
        self.serial.write(data)

    def disconnect(self) -> None:
        self.stop_event.set()
        current = self.serial
        self.serial = None
        if current:
            try:
                current.close()
            except Exception:
                pass
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=0.5)
        self.thread = None
