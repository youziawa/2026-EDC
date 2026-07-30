from __future__ import annotations

import math
from pathlib import Path
import tkinter as tk

from PIL import Image, ImageTk

from .telemetry import Telemetry


class FieldMap(tk.Canvas):
    FIELD_W = 400.0
    FIELD_H = 500.0

    def __init__(self, master, image_path: Path, **kwargs):
        super().__init__(master, background="#eef1f5", highlightthickness=0, **kwargs)
        self.source_image = Image.open(image_path).convert("RGB")
        self.display_image: ImageTk.PhotoImage | None = None
        self.last_image_size = (0, 0)
        self.telemetry: Telemetry | None = None
        self.car_trail: list[tuple[float, float]] = []
        self.drone_trail: list[tuple[float, float]] = []
        self.bind("<Configure>", lambda _event: self.redraw())

    def update_telemetry(self, telemetry: Telemetry) -> None:
        self.telemetry = telemetry
        if telemetry.car.x is not None and telemetry.car.y is not None:
            point = (telemetry.car.x, telemetry.car.y)
            if not self.car_trail or _distance(point, self.car_trail[-1]) > 2:
                self.car_trail.append(point)
                self.car_trail = self.car_trail[-600:]
        if telemetry.drone.x is not None and telemetry.drone.y is not None:
            point = (telemetry.drone.x, telemetry.drone.y)
            if not self.drone_trail or _distance(point, self.drone_trail[-1]) > 2:
                self.drone_trail.append(point)
                self.drone_trail = self.drone_trail[-600:]
        self.redraw()

    def clear_trails(self) -> None:
        self.car_trail.clear()
        self.drone_trail.clear()
        self.redraw()

    def _geometry(self) -> tuple[float, float, float, float]:
        width, height = max(self.winfo_width(), 100), max(self.winfo_height(), 100)
        ratio = self.source_image.width / self.source_image.height
        draw_h = min(height - 16, (width - 16) / ratio)
        draw_w = draw_h * ratio
        return (width - draw_w) / 2, (height - draw_h) / 2, draw_w, draw_h

    def _screen(self, x_cm: float, y_cm: float) -> tuple[float, float]:
        x0, y0, width, height = self._geometry()
        # Field interior in the supplied 611x762 print map.
        left = x0 + width * 5 / 611
        right = x0 + width * 605 / 611
        top = y0 + height * 6 / 762
        bottom = y0 + height * 756 / 762
        x = left + max(0, min(self.FIELD_W, x_cm)) / self.FIELD_W * (right - left)
        y = bottom - max(0, min(self.FIELD_H, y_cm)) / self.FIELD_H * (bottom - top)
        return x, y

    def redraw(self) -> None:
        self.delete("all")
        x0, y0, width, height = self._geometry()
        image_size = (max(1, round(width)), max(1, round(height)))
        if image_size != self.last_image_size:
            resized = self.source_image.resize(image_size, Image.Resampling.LANCZOS)
            self.display_image = ImageTk.PhotoImage(resized)
            self.last_image_size = image_size
        self.create_image(self.winfo_width() / 2, self.winfo_height() / 2, image=self.display_image)
        if not self.telemetry:
            return
        self._draw_trail(self.car_trail, "#17a673")
        self._draw_trail(self.drone_trail, "#4263eb")
        self._draw_vehicle(self.telemetry.car.x, self.telemetry.car.y, self.telemetry.car.heading, "#17a673", "车")
        self._draw_vehicle(self.telemetry.drone.x, self.telemetry.drone.y, self.telemetry.drone.heading, "#4263eb", "机")

    def _draw_trail(self, points: list[tuple[float, float]], color: str) -> None:
        if len(points) > 1:
            coords = [value for point in points for value in self._screen(*point)]
            self.create_line(*coords, fill=color, width=2, stipple="gray50")

    def _draw_vehicle(self, x, y, heading, color, label) -> None:
        if x is None or y is None:
            return
        sx, sy = self._screen(x, y)
        radius = 10
        self.create_oval(sx - radius, sy - radius, sx + radius, sy + radius, fill=color, outline="white", width=2)
        self.create_text(sx, sy, text=label, fill="white", font=("TkDefaultFont", 9, "bold"))
        angle = math.radians(heading)
        self.create_line(sx, sy, sx + 18 * math.cos(angle), sy - 18 * math.sin(angle), fill=color, width=3, arrow="last")


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
