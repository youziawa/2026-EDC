from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
import time
import tkinter as tk
from tkinter import messagebox, ttk

from .map_view import FieldMap
from .protocol import Frame, MSG_NAMES, Parser, encode
from .telemetry import Telemetry, task_execution_label
from .transport import SerialTransport, list_ports, serial_available

ROOT = Path(__file__).resolve().parents[1]


class GroundStationApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("LXS1 陆空协同地面站")
        self.geometry("1280x800")
        self.minsize(1050, 680)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.events: Queue = Queue()
        self.parsers = {
            "air": Parser(),
            "car": Parser(),
        }
        self.telemetry = Telemetry()
        self.air_transport = SerialTransport(self.events, "air", "飞机")
        self.car_transport = SerialTransport(self.events, "car", "小车")
        self.last_ui_update = 0.0
        self.port_map: dict[str, str] = {}
        self.vars = {name: tk.StringVar(value="—") for name in (
            "air_link", "car_link", "task", "elapsed", "drone_state", "drone_pos",
            "car_state", "car_pos", "speed", "path", "lap",
            "drop", "land", "frames", "errors",
        )}
        self.vars["air_link"].set("飞机：未连接")
        self.vars["car_link"].set("小车：未连接")
        self._build_style()
        self._build_ui()
        self._open_log()
        self._refresh_ports()
        self.after(30, self._poll)
        self.after(250, self._refresh_ui)
        self._event("地面站已启动；请分别连接飞机和小车串口")

    def _build_style(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=("Microsoft YaHei UI", 10))
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 18, "bold"), foreground="#172b4d")
        style.configure("Section.TLabel", font=("Microsoft YaHei UI", 11, "bold"), foreground="#334e68")
        style.configure("Value.TLabel", font=("Consolas", 12, "bold"), foreground="#102a43")

    def _build_ui(self) -> None:
        header = ttk.Frame(self, padding=(14, 10))
        header.pack(fill="x")
        ttk.Label(header, text="LXS1 陆空协同地面站", style="Title.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.vars["car_link"], padding=(18, 0)).pack(side="right")
        ttk.Label(header, textvariable=self.vars["air_link"], padding=(18, 0)).pack(side="right")

        connection = ttk.Frame(self, padding=(14, 0, 14, 8))
        connection.pack(fill="x")
        ttk.Label(connection, text="飞机串口").grid(row=0, column=0, sticky="w", pady=2)
        self.air_port_box = ttk.Combobox(connection, width=35, state="readonly")
        self.air_port_box.grid(row=0, column=1, padx=6, pady=2)
        self.air_baud_box = ttk.Combobox(
            connection, values=("115200", "57600", "230400"), width=9, state="readonly"
        )
        self.air_baud_box.set("115200")
        self.air_baud_box.grid(row=0, column=2, padx=3, pady=2)
        self.air_connect_button = ttk.Button(
            connection, text="连接飞机", command=lambda: self._toggle_connect("air")
        )
        self.air_connect_button.grid(row=0, column=3, padx=3, pady=2)

        ttk.Label(connection, text="小车串口").grid(row=1, column=0, sticky="w", pady=2)
        self.car_port_box = ttk.Combobox(connection, width=35, state="readonly")
        self.car_port_box.grid(row=1, column=1, padx=6, pady=2)
        self.car_baud_box = ttk.Combobox(
            connection, values=("115200", "57600", "230400"), width=9, state="readonly"
        )
        self.car_baud_box.set("115200")
        self.car_baud_box.grid(row=1, column=2, padx=3, pady=2)
        self.car_connect_button = ttk.Button(
            connection, text="连接小车", command=lambda: self._toggle_connect("car")
        )
        self.car_connect_button.grid(row=1, column=3, padx=3, pady=2)
        ttk.Button(connection, text="刷新串口", command=self._refresh_ports).grid(
            row=0, column=4, rowspan=2, padx=(12, 3)
        )
        ttk.Button(connection, text="清除轨迹", command=lambda: self.map.clear_trails()).grid(
            row=0, column=5, rowspan=2, padx=3
        )

        paned = ttk.Panedwindow(self, orient="horizontal")
        paned.pack(fill="both", expand=True, padx=14)
        map_frame = ttk.LabelFrame(paned, text=" 场地态势 ", padding=4)
        side = ttk.Frame(paned)
        paned.add(map_frame, weight=3)
        paned.add(side, weight=2)
        self.map = FieldMap(map_frame, ROOT / "assets" / "field_map.png")
        self.map.pack(fill="both", expand=True)

        task = ttk.LabelFrame(side, text=" 任务 ", padding=10)
        task.pack(fill="x", pady=(0, 7))
        self._grid_values(task, (
            ("任务状态", "task"), ("任务用时", "elapsed"), ("抛投", "drop"), ("降落", "land"),
        ))
        vehicles = ttk.LabelFrame(side, text=" 飞机 / 小车 ", padding=10)
        vehicles.pack(fill="x", pady=7)
        self._grid_values(vehicles, (
            ("飞机状态", "drone_state"), ("飞机坐标", "drone_pos"),
            ("小车状态", "car_state"), ("小车坐标", "car_pos"),
            ("速度", "speed"), ("里程 / 圈数", "path"),
        ))
        stats = ttk.LabelFrame(side, text=" 链路统计 ", padding=10)
        stats.pack(fill="x", pady=7)
        self._grid_values(stats, (("帧计数", "frames"), ("格式错误", "errors")))

        notebook = ttk.Notebook(side)
        notebook.pack(fill="both", expand=True, pady=(7, 0))
        event_tab = ttk.Frame(notebook)
        raw_tab = ttk.Frame(notebook)
        notebook.add(event_tab, text="事件")
        notebook.add(raw_tab, text="原始帧")
        self.event_text = self._text_area(event_tab)
        self.raw_text = self._text_area(raw_tab)

        status = ttk.Label(
            self,
            text="飞机坐标原点：H 点　飞机 +X 向地图上方 / +Y 向地图左方　单位：cm",
            anchor="w",
            padding=(14, 7),
        )
        status.pack(fill="x")

    def _grid_values(self, parent, rows) -> None:
        for row, (label, key) in enumerate(rows):
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Label(parent, textvariable=self.vars[key], style="Value.TLabel").grid(row=row, column=1, sticky="e", padx=(20, 0), pady=3)
        parent.columnconfigure(1, weight=1)

    @staticmethod
    def _text_area(parent):
        text = tk.Text(parent, height=8, wrap="none", state="disabled", font=("Consolas", 9), background="#f8fafc")
        scrollbar = ttk.Scrollbar(parent, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        return text

    def _open_log(self) -> None:
        log_dir = ROOT / "logs"
        log_dir.mkdir(exist_ok=True)
        path = log_dir / f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self.log_file = path.open("w", newline="", encoding="utf-8-sig")
        self.log = csv.writer(self.log_file)
        self.log.writerow(("time", "link", "src", "dst", "msg", "data_hex", "car_x", "car_y", "drone_x", "drone_y", "task_state"))

    def _refresh_ports(self) -> None:
        ports = list_ports()
        self.port_map = {f"{dev} — {desc}": dev for dev, desc in ports}
        choices = list(self.port_map)
        self.air_port_box["values"] = choices
        self.car_port_box["values"] = choices
        if ports:
            if self.air_port_box.get() not in self.port_map:
                self.air_port_box.current(0)
            if self.car_port_box.get() not in self.port_map:
                self.car_port_box.current(1 if len(ports) > 1 else 0)
        elif not serial_available():
            self.air_port_box.set("未安装 pyserial")
            self.car_port_box.set("未安装 pyserial")
        else:
            self.air_port_box.set("未发现串口")
            self.car_port_box.set("未发现串口")

    def _toggle_connect(self, channel: str) -> None:
        is_air = channel == "air"
        transport = self.air_transport if is_air else self.car_transport
        other = self.car_transport if is_air else self.air_transport
        port_box = self.air_port_box if is_air else self.car_port_box
        baud_box = self.air_baud_box if is_air else self.car_baud_box
        button = self.air_connect_button if is_air else self.car_connect_button
        link_key = "air_link" if is_air else "car_link"
        name = "飞机" if is_air else "小车"

        if transport.connected:
            transport.disconnect()
            button.configure(text=f"连接{name}")
            self.vars[link_key].set(f"{name}：未连接")
            self._event(f"{name}串口已断开")
            return
        display = port_box.get()
        port = self.port_map.get(display)
        if not port:
            messagebox.showwarning("无法连接", "请先选择有效串口。")
            return
        if other.connected and other.port == port:
            messagebox.showwarning("无法连接", "飞机和小车不能选择同一个串口。")
            return
        try:
            transport.connect(port, int(baud_box.get()))
            button.configure(text=f"断开{name}")
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))

    def _poll(self) -> None:
        try:
            while True:
                kind, channel, value = self.events.get_nowait()
                if kind == "bytes":
                    parser = self.parsers[channel]
                    for frame in parser.feed(value):
                        self._handle_frame(frame, channel)
                elif kind == "status":
                    self.vars[f"{channel}_link"].set(value)
                    self._event(value)
                elif kind == "error":
                    self._event(value)
                    self.vars[f"{channel}_link"].set(
                        ("飞机" if channel == "air" else "小车") + "：连接错误"
                    )
                elif kind == "disconnected":
                    name = "飞机" if channel == "air" else "小车"
                    self.vars[f"{channel}_link"].set(f"{name}：已断开")
                    button = self.air_connect_button if channel == "air" else self.car_connect_button
                    button.configure(text=f"连接{name}")
        except Empty:
            pass
        self.after(30, self._poll)

    def _handle_frame(self, frame: Frame, channel: str) -> None:
        packet = encode(frame)
        self._raw(f"RX-{channel.upper()}", packet)
        for event in self.telemetry.apply(frame):
            self._event(event)
        self.log.writerow((
            datetime.now().isoformat(timespec="milliseconds"), channel, f"0x{frame.src:02X}",
            f"0x{frame.dst:02X}", MSG_NAMES.get(frame.msg_id, f"0x{frame.msg_id:02X}"),
            frame.data.hex(" "), self.telemetry.car.x, self.telemetry.car.y,
            self.telemetry.drone.x, self.telemetry.drone.y, self.telemetry.task_state,
        ))

    def _refresh_ui(self) -> None:
        t = self.telemetry
        now = time.monotonic()
        if self.air_transport.connected:
            air_at = t.last_by_source.get(0x02, 0.0)
            air_age = now - air_at if air_at else float("inf")
            self.vars["air_link"].set(
                "飞机：在线" if air_age <= 2 else
                ("飞机：等待遥测" if air_age == float("inf") else f"飞机：超时 {air_age:.1f}s")
            )
        if self.car_transport.connected:
            car_at = t.last_by_source.get(0x04, 0.0)
            car_age = now - car_at if car_at else float("inf")
            self.vars["car_link"].set(
                "小车：在线" if car_age <= 2 else
                ("小车：等待遥测" if car_age == float("inf") else f"小车：超时 {car_age:.1f}s")
            )
        self.vars["task"].set(task_execution_label(t.task_mode, t.task_state))
        self.vars["elapsed"].set(f"{t.elapsed_ms / 1000:.1f} s")
        self.vars["drone_state"].set(t.drone.state + (" · 已解锁" if t.armed else " · 未解锁"))
        self.vars["drone_pos"].set(_position(t.drone.x, t.drone.y))
        self.vars["car_state"].set(t.car.state + (" · 循线有效" if t.line_valid else " · 循线无效"))
        self.vars["car_pos"].set(_position(t.car.x, t.car.y))
        self.vars["speed"].set(f"{t.car.speed:.0f} cm/s")
        self.vars["path"].set(f"{t.car.path:.0f} cm / {t.lap} 圈")
        self.vars["drop"].set(t.drop_state)
        self.vars["land"].set(f"{t.land_state} · {t.stable_time_s}s")
        self.vars["frames"].set(f"RX {t.frames_received}")
        self.vars["errors"].set(str(sum(parser.format_errors for parser in self.parsers.values())))
        self.map.update_telemetry(t)
        self.log_file.flush()
        self.after(250, self._refresh_ui)

    def _event(self, message: str) -> None:
        self._append(self.event_text, f"[{datetime.now():%H:%M:%S}] {message}\n", 600)

    def _raw(self, direction: str, packet: bytes) -> None:
        self._append(self.raw_text, f"{datetime.now():%H:%M:%S.%f}"[:-3] + f" {direction}  {packet.hex(' ').upper()}\n", 400)

    @staticmethod
    def _append(widget: tk.Text, text: str, max_lines: int) -> None:
        widget.configure(state="normal")
        widget.insert("end", text)
        line_count = int(widget.index("end-1c").split(".")[0])
        if line_count > max_lines:
            widget.delete("1.0", f"{line_count - max_lines}.0")
        widget.see("end")
        widget.configure(state="disabled")

    def _on_close(self) -> None:
        self.air_transport.disconnect()
        self.car_transport.disconnect()
        self.log_file.close()
        self.destroy()


def _position(x, y) -> str:
    return "—" if x is None or y is None else f"X {x:.0f} / Y {y:.0f} cm"


def main() -> None:
    app = GroundStationApp()
    app.mainloop()
