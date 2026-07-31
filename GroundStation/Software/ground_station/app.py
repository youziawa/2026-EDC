from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
import time
import tkinter as tk
from tkinter import messagebox, ttk

from .map_view import FieldMap
from .protocol import BROADCAST, DEVICE_NAMES, Frame, GROUND_STATION, MSG_NAMES, Parser, encode
from .simulator import Simulator
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
        self.parser = Parser()
        self.telemetry = Telemetry()
        self.transport = SerialTransport(self.events)
        self.simulator = Simulator(self.events)
        self.tx_frames = 0
        self.last_ui_update = 0.0
        self.port_map: dict[str, str] = {}
        self.vars = {name: tk.StringVar(value="—") for name in (
            "link", "task", "elapsed", "drone_state", "drone_pos", "altitude",
            "battery", "car_state", "car_pos", "speed", "path", "lap",
            "drop", "land", "frames", "errors",
        )}
        self.vars["link"].set("未连接")
        self._build_style()
        self._build_ui()
        self._open_log()
        self._refresh_ports()
        self.after(30, self._poll)
        self.after(250, self._refresh_ui)
        self._event("地面站已启动；可连接串口或进入演示模式")

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
        style.configure("Danger.TButton", foreground="#a61b1b")

    def _build_ui(self) -> None:
        header = ttk.Frame(self, padding=(14, 10))
        header.pack(fill="x")
        ttk.Label(header, text="LXS1 陆空协同地面站", style="Title.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.vars["link"], padding=(18, 0)).pack(side="right")

        connection = ttk.Frame(self, padding=(14, 0, 14, 8))
        connection.pack(fill="x")
        self.port_box = ttk.Combobox(connection, width=35, state="readonly")
        self.port_box.pack(side="left", padx=(0, 6))
        ttk.Button(connection, text="刷新串口", command=self._refresh_ports).pack(side="left", padx=3)
        self.baud_box = ttk.Combobox(connection, values=("115200", "57600", "230400"), width=9, state="readonly")
        self.baud_box.set("115200")
        self.baud_box.pack(side="left", padx=3)
        self.connect_button = ttk.Button(connection, text="连接", command=self._toggle_connect)
        self.connect_button.pack(side="left", padx=3)
        self.demo_mode_box = ttk.Combobox(
            connection,
            values=("动态起降演示", "抛投任务演示"),
            width=12,
            state="readonly",
        )
        self.demo_mode_box.set("动态起降演示")
        self.demo_mode_box.pack(side="left", padx=(14, 3))
        self.demo_button = ttk.Button(connection, text="启动演示", command=self._toggle_demo)
        self.demo_button.pack(side="left", padx=3)
        ttk.Button(connection, text="重置演示", command=self._reset_demo).pack(side="left", padx=3)
        ttk.Button(connection, text="清除轨迹", command=lambda: self.map.clear_trails()).pack(side="left", padx=3)
        ttk.Button(connection, text="终止任务", style="Danger.TButton", command=self._abort).pack(side="right")

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
            ("飞机状态", "drone_state"), ("飞机坐标", "drone_pos"), ("高度", "altitude"),
            ("电池", "battery"), ("小车状态", "car_state"), ("小车坐标", "car_pos"),
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

        status = ttk.Label(self, text="坐标原点：场地左下角　X 向右 / Y 向上　单位：cm", anchor="w", padding=(14, 7))
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
        self.log.writerow(("time", "src", "dst", "msg", "data_hex", "car_x", "car_y", "drone_x", "drone_y", "task_state"))

    def _refresh_ports(self) -> None:
        ports = list_ports()
        self.port_map = {f"{dev} — {desc}": dev for dev, desc in ports}
        self.port_box["values"] = list(self.port_map)
        if ports:
            self.port_box.current(0)
        elif not serial_available():
            self.port_box.set("未安装 pyserial（演示模式可用）")
        else:
            self.port_box.set("未发现串口")

    def _toggle_connect(self) -> None:
        if self.transport.connected:
            self.transport.disconnect()
            self.connect_button.configure(text="连接")
            self.vars["link"].set("未连接")
            self._event("串口已断开")
            return
        display = self.port_box.get()
        port = self.port_map.get(display)
        if not port:
            messagebox.showwarning("无法连接", "请先选择有效串口；没有硬件时可使用演示模式。")
            return
        try:
            self.simulator.stop()
            self.transport.connect(port, int(self.baud_box.get()))
            self.connect_button.configure(text="断开")
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))

    def _toggle_demo(self) -> None:
        if self.simulator.running:
            self.simulator.stop()
            self.demo_button.configure(text="启动演示")
            self.vars["link"].set("演示已停止")
            self._event("演示模式已停止")
        else:
            self.transport.disconnect()
            self.connect_button.configure(text="连接")
            self.telemetry = Telemetry()
            self.parser = Parser()
            self.map.clear_trails()
            self.simulator.start(self._selected_demo_mode())
            self.demo_button.configure(text="停止演示")

    def _selected_demo_mode(self) -> int:
        return 1 if self.demo_mode_box.get() == "抛投任务演示" else 2

    def _reset_demo(self) -> None:
        if self.transport.connected:
            messagebox.showwarning("无法重置", "当前正在使用真实串口，请先断开串口。")
            return
        self.simulator.stop()
        while True:
            try:
                self.events.get_nowait()
            except Empty:
                break
        self.telemetry = Telemetry()
        self.parser = Parser()
        self.tx_frames = 0
        self.map.clear_trails()
        self.simulator.start(self._selected_demo_mode())
        self.demo_button.configure(text="停止演示")
        self._event("演示已重置：小车从 A 点按 A→B→C→D→A 顺时针重新出发")

    def _abort(self) -> None:
        if not (self.transport.connected or self.simulator.running):
            messagebox.showwarning("无法发送", "当前没有连接。")
            return
        if not messagebox.askyesno("确认终止", "确定广播 TASK_ABORT 吗？\n该操作会终止当前任务。", icon="warning"):
            return
        packet = encode(
            Frame(
                GROUND_STATION,
                BROADCAST,
                0x11,
                bytes((self.telemetry.run_id & 0xFF, 1)),
            )
        )
        try:
            if self.transport.connected:
                self.transport.write(packet)
            self.tx_frames += 1
            self._event(
                f"已广播 TASK_ABORT：run_id={self.telemetry.run_id}，reason=1"
            )
            self._raw("TX", packet)
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def _poll(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "bytes":
                    for frame in self.parser.feed(value):
                        self._handle_frame(frame)
                elif kind == "status":
                    self.vars["link"].set(value)
                    self._event(value)
                elif kind == "error":
                    self._event(value)
                    self.vars["link"].set("连接错误")
                elif kind == "disconnected":
                    self.vars["link"].set("已断开")
                    self.connect_button.configure(text="连接")
        except Empty:
            pass
        self.after(30, self._poll)

    def _handle_frame(self, frame: Frame) -> None:
        packet = encode(frame)
        self._raw("RX", packet)
        for event in self.telemetry.apply(frame):
            self._event(event)
        self.log.writerow((
            datetime.now().isoformat(timespec="milliseconds"), f"0x{frame.src:02X}",
            f"0x{frame.dst:02X}", MSG_NAMES.get(frame.msg_id, f"0x{frame.msg_id:02X}"),
            frame.data.hex(" "), self.telemetry.car.x, self.telemetry.car.y,
            self.telemetry.drone.x, self.telemetry.drone.y, self.telemetry.task_state,
        ))

    def _refresh_ui(self) -> None:
        t = self.telemetry
        now = time.monotonic()
        age = now - t.last_frame_at if t.last_frame_at else float("inf")
        if self.transport.connected:
            self.vars["link"].set("在线" if age <= 2 else f"遥测超时 {age:.1f}s")
        self.vars["task"].set(task_execution_label(t.task_mode, t.task_state))
        self.vars["elapsed"].set(f"{t.elapsed_ms / 1000:.1f} s")
        self.vars["drone_state"].set(t.drone.state + (" · 已解锁" if t.armed else " · 未解锁"))
        self.vars["drone_pos"].set(_position(t.drone.x, t.drone.y))
        self.vars["altitude"].set("—" if t.drone.z is None else f"{t.drone.z:.0f} cm")
        self.vars["battery"].set("—" if not t.battery_mv else f"{t.battery_mv / 1000:.2f} V")
        self.vars["car_state"].set(t.car.state + (" · 循线有效" if t.line_valid else " · 循线无效"))
        self.vars["car_pos"].set(_position(t.car.x, t.car.y))
        self.vars["speed"].set(f"{t.car.speed:.0f} cm/s")
        self.vars["path"].set(f"{t.car.path:.0f} cm / {t.lap} 圈")
        self.vars["drop"].set(t.drop_state)
        self.vars["land"].set(f"{t.land_state} · {t.stable_time_s}s")
        self.vars["frames"].set(f"RX {t.frames_received} / TX {self.tx_frames}")
        self.vars["errors"].set(str(self.parser.format_errors))
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
        self.simulator.stop()
        self.transport.disconnect()
        self.log_file.close()
        self.destroy()


def _position(x, y) -> str:
    return "—" if x is None or y is None else f"X {x:.0f} / Y {y:.0f} cm"


def main() -> None:
    app = GroundStationApp()
    app.mainloop()
