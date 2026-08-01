from __future__ import annotations

import math

TRACK_RADIUS_CM = 75.0
TRACK_STRAIGHT_CM = 150.0
TRACK_LENGTH_CM = 2 * TRACK_STRAIGHT_CM + 2 * math.pi * TRACK_RADIUS_CM

# 飞机SLAM原点H在地面站场地坐标中的位置。
DRONE_HOME_X_CM = 112.5
DRONE_HOME_Y_CM = 112.5


def drone_position_from_home(x_cm: float, y_cm: float) -> tuple[float, float]:
    """Convert aircraft H-relative coordinates to field-map coordinates.

    Aircraft +X points upward on the printed map; aircraft +Y points left.
    """
    return DRONE_HOME_X_CM - y_cm, DRONE_HOME_Y_CM + x_cm


def position_from_mileage(path_cm: float) -> tuple[float, float, float]:
    """Convert cumulative mileage to clockwise A→B→C→D→A field pose.

    Returns x_cm, y_cm and heading_deg. Mileage may exceed one lap.
    """
    distance = max(0.0, path_cm) % TRACK_LENGTH_CM
    radius = TRACK_RADIUS_CM
    straight = TRACK_STRAIGHT_CM
    center_x = 225.0
    a_y, b_y = 200.0, 350.0

    # A → B: left straight, heading north.
    if distance < straight:
        return center_x - radius, a_y + distance, 90.0
    distance -= straight

    # B → C: upper semicircle.
    arc = math.pi * radius
    if distance < arc:
        angle = math.pi - distance / radius
        return (
            center_x + radius * math.cos(angle),
            b_y + radius * math.sin(angle),
            math.degrees(angle - math.pi / 2),
        )
    distance -= arc

    # C → D: right straight, heading south.
    if distance < straight:
        return center_x + radius, b_y - distance, -90.0
    distance -= straight

    # D → A: lower semicircle.
    angle = -distance / radius
    return (
        center_x + radius * math.cos(angle),
        a_y + radius * math.sin(angle),
        math.degrees(angle - math.pi / 2),
    )
