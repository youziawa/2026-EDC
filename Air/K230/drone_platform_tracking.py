"""K230 红色圆点追踪。

飞机沿原航点路线到达视觉接管区后，F407 发送 TASK_START。K230 随后
在 RGB565 图像中分割红色区域，以面积、宽高比、填充率和连续帧确认
标靶中心的红色圆点，并通过 LXS1 VISION_TARGET 发送像素偏移。

本程序不再执行灰度霍夫圆、圆环采样或十字线验证。
"""

import time
import os
import gc
import math
import machine

from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbUart import YbUart


# ======================== 图像与红点参数 ========================
IMG_W = 400
IMG_H = 240
CAMERA_CX_PX = IMG_W // 2
CAMERA_CY_PX = IMG_H // 2

# 调试阶段使用纯 IDE 虚拟显示，画布与相机同尺寸，避免LCD通道和缩放占用。
DISPLAY_W = IMG_W
DISPLAY_H = IMG_H
DISPLAY_X = 0
DISPLAY_Y = 0

# RGB565 的 find_blobs 使用 LAB 六元组：(L_min,L_max,A_min,A_max,B_min,B_max)。
# 红色在 LAB 中 A 通道为明显正值。现场若光照差异较大，优先只调整此项。
RED_THRESHOLDS = [(15, 85, 25, 127, -20, 90)]

RED_MIN_PIXELS = 12
RED_MIN_AREA = 16
RED_MIN_SIZE_PX = 4
RED_MAX_SIZE_PX = 90
RED_MIN_ASPECT = 0.55
RED_MIN_FILL = 0.28

LOCK_CONF = 0.52
TRACK_CONF = 0.45             # 与 F407 VISION_MIN_CONFIDENCE=450 对齐。
LOCK_FRAMES = 2
LOST_FRAMES = 8
TRACK_WINDOW_PX = 110
FILTER_ALPHA = 0.65

DETECT_INTERVAL_MS = 50
HEARTBEAT_PERIOD_MS = 1000
PRINT_INTERVAL_MS = 1000
PREVIEW_INTERVAL_MS = 100
GC_INTERVAL_FRAMES = 60
DEBUG_DISPLAY = True           # 调试阶段在 CanMV IDE 中显示彩色画面和红点标记。
FLIGHT_DISPLAY = False         # 飞行阶段关闭VIRT绘图，避免显示与实时取图争用。
DEBUG_PRINT = True


# ======================== LXS1 通信 ========================
UART_BAUD = 115200
LXS1_NODE_AIR_K230 = 0x03
LXS1_NODE_AIR_MCU = 0x02
LXS1_MSG_HEARTBEAT = 0x02
LXS1_MSG_TASK_START = 0x10
LXS1_MSG_VISION_TARGET = 0x40
LXS1_MAX_DATA = 64
LXS1_TARGET_KIND_PLATFORM = 2
LXS1_VISION_TARGET_DATA_LEN = 12

VISION_ACQUIRE = 1
VISION_TRACK = 2
VISION_LOST = 3


def clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def _append_u16_le(data, value):
    value = int(value) & 0xFFFF
    data.append(value & 0xFF)
    data.append((value >> 8) & 0xFF)


def _append_i16_le(data, value):
    _append_u16_le(data, int(value))


def make_lxs1_frame(msg_id, data):
    if len(data) > LXS1_MAX_DATA:
        raise ValueError("LXS1 payload too long")
    frame = bytearray()
    frame.append(0xAA)
    frame.append(0x55)
    frame.append(3 + len(data))
    frame.append(LXS1_NODE_AIR_K230)
    frame.append(LXS1_NODE_AIR_MCU)
    frame.append(msg_id)
    frame.extend(data)
    frame.append(0x0D)
    frame.append(0x0A)
    return frame


def make_heartbeat_frame():
    return make_lxs1_frame(LXS1_MSG_HEARTBEAT, bytearray())


def make_vision_target_frame(valid, cx, cy, radius, confidence, mode):
    """保持 F407 已使用的 12 字节 VISION_TARGET 数据布局。"""
    data = bytearray()
    if valid:
        dx_px = int(cx - CAMERA_CX_PX)
        dy_px = int(cy - CAMERA_CY_PX)
        radius_px = max(1, int(radius))
        valid_field = 1
    else:
        dx_px = 0
        dy_px = 0
        radius_px = 0
        valid_field = 0

    confidence_permille = int(clamp(confidence * 1000.0, 0, 1000))
    data.append(valid_field)
    data.append(LXS1_TARGET_KIND_PLATFORM)
    _append_u16_le(data, confidence_permille)
    _append_i16_le(data, dx_px)
    _append_i16_le(data, dy_px)
    _append_i16_le(data, radius_px)
    _append_i16_le(data, mode)

    if len(data) != LXS1_VISION_TARGET_DATA_LEN:
        raise ValueError("VISION_TARGET data length must be 12")
    return make_lxs1_frame(LXS1_MSG_VISION_TARGET, data)


command_rx_buffer = bytearray()


def poll_vision_start_command(uart):
    """非阻塞解析 F407 发来的平台 TASK_START。"""
    chunk = uart.read(64, decode=False)
    if chunk:
        command_rx_buffer.extend(chunk)

    while len(command_rx_buffer) >= 3:
        if command_rx_buffer[0] != 0xAA or command_rx_buffer[1] != 0x55:
            del command_rx_buffer[0]
            continue

        declared_length = command_rx_buffer[2]
        if declared_length < 3 or declared_length > 3 + LXS1_MAX_DATA:
            del command_rx_buffer[0]
            continue

        total_length = 3 + declared_length + 2
        if len(command_rx_buffer) < total_length:
            break
        if (command_rx_buffer[total_length - 2] != 0x0D or
                command_rx_buffer[total_length - 1] != 0x0A):
            del command_rx_buffer[0]
            continue

        src = command_rx_buffer[3]
        dst = command_rx_buffer[4]
        msg_id = command_rx_buffer[5]
        data_len = declared_length - 3
        platform_requested = (
            src == LXS1_NODE_AIR_MCU and
            dst == LXS1_NODE_AIR_K230 and
            msg_id == LXS1_MSG_TASK_START and
            data_len == 1 and
            command_rx_buffer[6] == LXS1_TARGET_KIND_PLATFORM
        )
        del command_rx_buffer[:total_length]
        if platform_requested:
            return True

    if len(command_rx_buffer) > 128:
        del command_rx_buffer[:-2]
    return False


# ======================== 红色圆点识别 ========================
def _blob_value(blob, name):
    value = getattr(blob, name)
    return value() if callable(value) else value


def evaluate_red_blob(blob):
    """返回 (候选, 置信度, cx, cy, radius, pixels, w, h, fill)。"""
    cx = int(_blob_value(blob, "cx"))
    cy = int(_blob_value(blob, "cy"))
    width = int(_blob_value(blob, "w"))
    height = int(_blob_value(blob, "h"))
    pixels = int(_blob_value(blob, "pixels"))
    area = max(1, width * height)

    if (width < RED_MIN_SIZE_PX or height < RED_MIN_SIZE_PX or
            width > RED_MAX_SIZE_PX or height > RED_MAX_SIZE_PX):
        return False, 0.0, cx, cy, 0, pixels, width, height, 0.0

    aspect = min(width, height) / float(max(width, height))
    fill = pixels / float(area)
    if aspect < RED_MIN_ASPECT or fill < RED_MIN_FILL:
        return False, 0.0, cx, cy, 0, pixels, width, height, fill

    # 圆形理论包围盒填充率约0.785；光斑、遮挡会使其下降。
    # 基本门槛已经排除了过小、过扁和过稀疏的噪声。置信度直接使用
    # 圆度、填充率和可见像素量，避免小而清晰的中心红点因尺寸项被压低。
    aspect_score = aspect
    fill_score = clamp(fill / 0.65, 0.0, 1.0)
    size_score = clamp(pixels / 40.0, 0.0, 1.0)
    confidence = 0.55 * aspect_score + 0.35 * fill_score + 0.10 * size_score
    radius = max(1, int(math.sqrt(pixels / math.pi) + 0.5))
    return True, confidence, cx, cy, radius, pixels, width, height, fill


def find_red_dot(img, track_cx, track_cy):
    """选择最像圆点、且在 TRACK 时最接近上一位置的红色连通域。"""
    blobs = img.find_blobs(
        RED_THRESHOLDS,
        x_stride=2, y_stride=2,
        pixels_threshold=RED_MIN_PIXELS,
        area_threshold=RED_MIN_AREA,
        merge=True, margin=3
    )

    best = None
    best_rank = -1000.0
    accepted_count = 0
    for blob in blobs:
        result = evaluate_red_blob(blob)
        accepted, confidence, cx, cy, radius, pixels, width, height, fill = result
        if not accepted:
            continue
        accepted_count += 1

        distance = 0.0
        if track_cx is not None and track_cy is not None:
            dx = cx - track_cx
            dy = cy - track_cy
            distance = math.sqrt(dx * dx + dy * dy)
            if distance > TRACK_WINDOW_PX:
                continue

        # 首次捕获优先结构置信度和像素数；跟踪时同时惩罚位置跳变。
        rank = confidence + min(pixels, 200) / 1000.0
        rank -= distance / (TRACK_WINDOW_PX * 2.0)
        if rank > best_rank:
            best_rank = rank
            best = result
    return best, len(blobs), accepted_count


# ======================== 硬件初始化 ========================
os.exitpoint(os.EXITPOINT_ENABLE)
print("[BOOT] red tracker start")

uart = YbUart(baudrate=UART_BAUD)
print("[BOOT] uart ready")
sensor = Sensor(id=2, width=1280, height=720, fps=90)
sensor.reset()
sensor.set_framesize(width=IMG_W, height=IMG_H)
sensor.set_pixformat(Sensor.RGB565)
print("[BOOT] sensor configured RGB565 400x240")

# VIRT 是 CanMV IDE 专用虚拟显示设备；不依赖 ST7701 实体屏。
Display.init(Display.VIRT, width=DISPLAY_W, height=DISPLAY_H, fps=10)
print("[BOOT] IDE virtual display ready")
MediaManager.init()
print("[BOOT] media manager ready")
sensor.run()
print("[BOOT] sensor running; waiting for preview")

clock = time.clock()
vision_enabled = False
vision_state = VISION_ACQUIRE
lock_count = 0
lost_count = 0
filtered_cx = None
filtered_cy = None
filtered_radius = None
track_cx = None
track_cy = None

last_detect_ms = time.ticks_ms() - DETECT_INTERVAL_MS
last_heartbeat_ms = time.ticks_ms()
last_print_ms = time.ticks_ms()
last_preview_ms = time.ticks_ms() - PREVIEW_INTERVAL_MS
frame_count = 0
detect_count = 0
last_blob_count = 0
last_accepted_count = 0
last_confidence = 0.0
last_event = "WAIT_START"


try:
    while True:
        clock.tick()
        frame_count += 1
        now_ms = time.ticks_ms()

        if not vision_enabled:
            if poll_vision_start_command(uart):
                vision_enabled = True
                vision_state = VISION_ACQUIRE
                lock_count = 0
                lost_count = 0
                filtered_cx = None
                filtered_cy = None
                filtered_radius = None
                track_cx = None
                track_cy = None
                last_event = "RED_ENABLED"
                last_detect_ms = now_ms - DETECT_INTERVAL_MS
            else:
                if time.ticks_diff(now_ms, last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS:
                    uart.send(bytes(make_heartbeat_frame()))
                    last_heartbeat_ms = now_ms
                # 调试预览不依赖飞机任务启动：上电后即可在 IDE 中观察
                # 彩色画面、画面中心以及当前红点候选。
                if (DEBUG_DISPLAY and
                        time.ticks_diff(now_ms, last_preview_ms) >=
                        PREVIEW_INTERVAL_MS):
                    last_preview_ms = now_ms
                    preview_img = sensor.snapshot()
                    preview_img.draw_cross(
                        CAMERA_CX_PX, CAMERA_CY_PX,
                        color=(0, 255, 0), thickness=1)
                    try:
                        preview_target, _, _ = find_red_dot(
                            preview_img, None, None)
                        if preview_target is not None:
                            (_, _, preview_cx, preview_cy, preview_radius,
                             _, _, _, _) = preview_target
                            preview_confidence = preview_target[1]
                            preview_color = ((0, 255, 0)
                                             if preview_confidence >= LOCK_CONF
                                             else (255, 128, 0))
                            preview_img.draw_circle(
                                preview_cx, preview_cy, preview_radius,
                                color=preview_color, thickness=2)
                            preview_img.draw_cross(
                                preview_cx, preview_cy,
                                color=(255, 255, 255), thickness=2)
                    except Exception as preview_exc:
                        print("red preview failed:", preview_exc)
                    Display.show_image(
                        preview_img, x=DISPLAY_X, y=DISPLAY_Y)
                time.sleep_ms(10)
                continue

        # 持续排空 F407 的重复启动帧，避免 UART RX 堆积。
        poll_vision_start_command(uart)
        if time.ticks_diff(now_ms, last_detect_ms) < DETECT_INTERVAL_MS:
            time.sleep_ms(2)
            continue
        last_detect_ms = now_ms

        img = sensor.snapshot()
        detect_count += 1
        detector_failed = False
        try:
            target, last_blob_count, last_accepted_count = find_red_dot(
                img, track_cx, track_cy)
            # A bend or fast relative motion can move the dot outside the
            # local tracking window in one frame. Retry the same image over
            # the full field before declaring a miss.
            if target is None and track_cx is not None:
                target, last_blob_count, last_accepted_count = find_red_dot(
                    img, None, None)
        except Exception as detector_exc:
            print("red detector frame failed:", detector_exc)
            target = None
            last_blob_count = 0
            last_accepted_count = 0
            detector_failed = True

        detected = False
        confidence = 0.999 if detector_failed else 0.0
        cx = CAMERA_CX_PX
        cy = CAMERA_CY_PX
        radius = 0
        if target is not None:
            _, confidence, cx, cy, radius, _, _, _, _ = target
            detected = True
        last_confidence = confidence

        packet_valid = False
        if vision_state == VISION_ACQUIRE:
            if detected and confidence >= LOCK_CONF:
                lock_count = min(lock_count + 1, LOCK_FRAMES)
            else:
                lock_count = 0

            if lock_count >= LOCK_FRAMES:
                filtered_cx = cx
                filtered_cy = cy
                filtered_radius = radius
                track_cx = cx
                track_cy = cy
                lost_count = 0
                vision_state = VISION_TRACK
                packet_valid = True
                last_event = "RED_LOCKED"

        elif vision_state == VISION_TRACK:
            if detected and confidence >= TRACK_CONF:
                lost_count = 0
                track_cx = cx
                track_cy = cy
                filtered_cx = int(
                    FILTER_ALPHA * cx + (1.0 - FILTER_ALPHA) * filtered_cx)
                filtered_cy = int(
                    FILTER_ALPHA * cy + (1.0 - FILTER_ALPHA) * filtered_cy)
                filtered_radius = int(
                    FILTER_ALPHA * radius +
                    (1.0 - FILTER_ALPHA) * filtered_radius)
                packet_valid = True
            else:
                lost_count += 1
                if lost_count >= LOST_FRAMES:
                    vision_state = VISION_LOST
                    filtered_cx = None
                    filtered_cy = None
                    filtered_radius = None
                    track_cx = None
                    track_cy = None
                    lock_count = 0
                    last_event = "RED_LOST"

        if packet_valid:
            packet = make_vision_target_frame(
                1, filtered_cx, filtered_cy, filtered_radius,
                confidence, VISION_TRACK)
        else:
            packet = make_vision_target_frame(
                0, CAMERA_CX_PX, CAMERA_CY_PX, 0,
                confidence, vision_state)

        uart.send(bytes(packet))
        # 视觉帧本身维持链路在线，避免随后紧跟心跳形成串口突发。
        last_heartbeat_ms = time.ticks_ms()

        if vision_state == VISION_LOST:
            vision_state = VISION_ACQUIRE
            lost_count = 0

        if DEBUG_DISPLAY and FLIGHT_DISPLAY:
            img.draw_cross(CAMERA_CX_PX, CAMERA_CY_PX,
                           color=(0, 255, 0), thickness=1)
            if packet_valid:
                img.draw_circle(filtered_cx, filtered_cy, filtered_radius,
                                color=(0, 255, 0), thickness=2)
                img.draw_cross(filtered_cx, filtered_cy,
                               color=(255, 255, 255), thickness=2)
            Display.show_image(img, x=DISPLAY_X, y=DISPLAY_Y)

        print_now_ms = time.ticks_ms()
        if (DEBUG_PRINT and
                time.ticks_diff(print_now_ms, last_print_ms) >= PRINT_INTERVAL_MS):
            elapsed_ms = max(1, time.ticks_diff(print_now_ms, last_print_ms))
            detect_hz = detect_count * 1000.0 / elapsed_ms
            print("RED fps=", clock.fps(),
                  "detect_hz=", detect_hz,
                  "blobs=", last_blob_count,
                  "accepted=", last_accepted_count,
                  "conf=", last_confidence,
                  "state=", vision_state,
                  "lock=", lock_count,
                  "lost=", lost_count,
                  "event=", last_event)
            detect_count = 0
            last_print_ms = print_now_ms

        if frame_count % GC_INTERVAL_FRAMES == 0:
            gc.collect()

except Exception as exc:
    # 主循环异常不再复位K230。日志中的 conf=999 已证明复位会造成
    # USART3错误并永久错过本次追车；这里转入不含显示和命令解析的
    # 轻量恢复循环，继续完成红点锁定和TRACK帧发送。
    print("PRIMARY red tracking exception; enter recovery:", exc)
    recovery_lock_count = 0
    recovery_lost_count = 0
    recovery_locked = False
    recovery_filtered_cx = None
    recovery_filtered_cy = None
    recovery_filtered_radius = None

    try:
        uart.send(bytes(make_vision_target_frame(
            0, CAMERA_CX_PX, CAMERA_CY_PX, 0, 0.998, VISION_LOST)))
        time.sleep_ms(20)
    except Exception as report_exc:
        print("PRIMARY exception report failed:", report_exc)

    while True:
        try:
            recovery_img = sensor.snapshot()
            recovery_target, _, _ = find_red_dot(
                recovery_img, recovery_filtered_cx, recovery_filtered_cy)
            if recovery_target is None and recovery_filtered_cx is not None:
                recovery_target, _, _ = find_red_dot(
                    recovery_img, None, None)

            if recovery_target is not None:
                (_, recovery_confidence, recovery_cx, recovery_cy,
                 recovery_radius, _, _, _, _) = recovery_target
            else:
                recovery_confidence = 0.0

            recovery_required_conf = (
                TRACK_CONF if recovery_locked else LOCK_CONF)
            if (recovery_target is not None and
                    recovery_confidence >= recovery_required_conf):
                recovery_lost_count = 0
                if not recovery_locked:
                    recovery_lock_count = min(
                        recovery_lock_count + 1, LOCK_FRAMES)
                if recovery_filtered_cx is None:
                    recovery_filtered_cx = recovery_cx
                    recovery_filtered_cy = recovery_cy
                    recovery_filtered_radius = recovery_radius
                else:
                    recovery_filtered_cx = int(
                        FILTER_ALPHA * recovery_cx +
                        (1.0 - FILTER_ALPHA) * recovery_filtered_cx)
                    recovery_filtered_cy = int(
                        FILTER_ALPHA * recovery_cy +
                        (1.0 - FILTER_ALPHA) * recovery_filtered_cy)
                    recovery_filtered_radius = int(
                        FILTER_ALPHA * recovery_radius +
                        (1.0 - FILTER_ALPHA) * recovery_filtered_radius)

                if (not recovery_locked and
                        recovery_lock_count >= LOCK_FRAMES):
                    recovery_locked = True

                if recovery_locked:
                    recovery_packet = make_vision_target_frame(
                        1,
                        recovery_filtered_cx,
                        recovery_filtered_cy,
                        recovery_filtered_radius,
                        recovery_confidence,
                        VISION_TRACK)
                else:
                    recovery_packet = make_vision_target_frame(
                        0, CAMERA_CX_PX, CAMERA_CY_PX, 0,
                        recovery_confidence, VISION_ACQUIRE)
            else:
                if recovery_locked:
                    recovery_lost_count += 1
                    recovery_packet = make_vision_target_frame(
                        0, CAMERA_CX_PX, CAMERA_CY_PX, 0,
                        recovery_confidence, VISION_TRACK)
                    if recovery_lost_count >= LOST_FRAMES:
                        recovery_locked = False
                        recovery_lock_count = 0
                        recovery_lost_count = 0
                        recovery_filtered_cx = None
                        recovery_filtered_cy = None
                        recovery_filtered_radius = None
                else:
                    recovery_lock_count = 0
                    recovery_filtered_cx = None
                    recovery_filtered_cy = None
                    recovery_filtered_radius = None
                    recovery_packet = make_vision_target_frame(
                        0, CAMERA_CX_PX, CAMERA_CY_PX, 0,
                        recovery_confidence, VISION_ACQUIRE)

            uart.send(bytes(recovery_packet))
            time.sleep_ms(DETECT_INTERVAL_MS)

        except Exception as recovery_exc:
            # conf=997表示恢复循环自身出现了单帧异常；不退出、不复位，
            # 下一帧继续尝试。
            print("RECOVERY red frame exception:", recovery_exc)
            try:
                uart.send(bytes(make_vision_target_frame(
                    0, CAMERA_CX_PX, CAMERA_CY_PX, 0,
                    0.997, VISION_LOST)))
            except Exception:
                pass
            time.sleep_ms(100)

finally:
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
    uart.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
