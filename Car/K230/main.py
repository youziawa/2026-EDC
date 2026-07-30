"""亚博 K230 小车黑线视觉主程序。"""

import gc
import os
import time

import image as image_module
from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_1, Sensor

import vision_config as cfg
from line_vision import LineDetector
from lxs1_uart import VisionLineSender


def _ticks_add(value, delta):
    try:
        return time.ticks_add(value, delta)
    except AttributeError:
        return value + delta


def setup_uart():
    channel = cfg.UART_CHANNEL
    uart_id = getattr(UART, "UART%d" % channel)
    tx_function = getattr(FPIOA, "UART%d_TXD" % channel)
    rx_function = getattr(FPIOA, "UART%d_RXD" % channel)

    fpioa = FPIOA()
    fpioa.set_function(cfg.UART_TX_PIN, tx_function)
    fpioa.set_function(cfg.UART_RX_PIN, rx_function)

    return UART(
        uart_id,
        baudrate=cfg.UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0,
    )


def setup_display(sensor):
    if not cfg.DISPLAY_ENABLED:
        return False

    try:
        bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        try:
            Display.bind_layer(
                **bind_info,
                layer=Display.LAYER_VIDEO1,
            )
        except TypeError:
            # 兼容 bind_info 返回三元组的旧版固件。
            Display.bind_layer(
                src=bind_info[0],
                rect=bind_info[1],
                pix_format=bind_info[2],
                layer=Display.LAYER_VIDEO1,
            )

        display_type = getattr(Display, cfg.DISPLAY_TYPE)
        if cfg.DISPLAY_TYPE == "VIRT":
            Display.init(
                display_type,
                width=cfg.DISPLAY_WIDTH,
                height=cfg.DISPLAY_HEIGHT,
                osd_num=1,
                fps=cfg.DISPLAY_FPS,
                to_ide=cfg.DISPLAY_TO_IDE,
            )
        else:
            # 亚博 ST7701 示例不传 fps，兼容其定制固件的面板初始化接口。
            Display.init(
                display_type,
                width=cfg.DISPLAY_WIDTH,
                height=cfg.DISPLAY_HEIGHT,
                osd_num=1,
                to_ide=cfg.DISPLAY_TO_IDE,
            )
        try:
            sensor._set_chn_fps(
                chn=CAM_CHN_ID_0,
                fps=Display.fps(),
            )
        except Exception:
            pass
        print(
            "Display bound: %s %dx%d, sensor CH0 -> VIDEO1"
            % (cfg.DISPLAY_TYPE, cfg.DISPLAY_WIDTH, cfg.DISPLAY_HEIGHT)
        )
        return True
    except Exception as error:
        # 显示故障不能阻止视觉和串口继续运行。
        print("Display disabled:", error)
        try:
            Display.deinit()
        except Exception:
            pass
        return False


def setup_camera():
    try:
        sensor = Sensor(
            width=cfg.CAMERA_MODE_WIDTH,
            height=cfg.CAMERA_MODE_HEIGHT,
            fps=cfg.CAMERA_MODE_FPS,
        )
    except (TypeError, ValueError):
        # 兼容不支持带参数 Sensor 构造函数的旧版亚博固件。
        sensor = Sensor()

    sensor.reset()
    # 通道0由硬件直接送屏；通道1只供视觉算法抓取。
    sensor.set_framesize(
        width=cfg.DISPLAY_WIDTH,
        height=cfg.DISPLAY_HEIGHT,
        chn=CAM_CHN_ID_0,
    )
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    sensor.set_framesize(
        width=cfg.IMAGE_WIDTH,
        height=cfg.IMAGE_HEIGHT,
        chn=CAM_CHN_ID_1,
    )
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

    try:
        sensor.set_hmirror(cfg.CAMERA_HMIRROR)
        sensor.set_vflip(cfg.CAMERA_VFLIP)
    except Exception:
        pass

    if cfg.MANUAL_EXPOSURE_US > 0:
        try:
            sensor.auto_exposure(False)
        except Exception:
            pass

    return sensor


def start_camera(sensor):
    MediaManager.init()
    sensor.run()
    if cfg.MANUAL_EXPOSURE_US > 0:
        try:
            sensor.exposure(float(cfg.MANUAL_EXPOSURE_US))
        except Exception:
            pass

    # 丢弃启动阶段曝光尚未稳定的图像。
    for _ in range(10):
        sensor.snapshot(chn=CAM_CHN_ID_1)
        time.sleep_ms(10)


def run():
    uart = None
    sensor = None
    sender = None
    display_ready = False
    display_buffers = None
    display_buffer_index = 0
    frame_count = 0
    diag_frame_count = 0
    measured_fps = 0.0

    try:
        sensor = setup_camera()
        display_ready = setup_display(sensor)

        try:
            uart = setup_uart()
            sender = VisionLineSender(uart, cfg.LXS1_SRC, cfg.LXS1_DST)
            print(
                "UART%d initialized: %d baud"
                % (cfg.UART_CHANNEL, cfg.UART_BAUDRATE)
            )
        except Exception as error:
            # 通用 CanMV 固件可能占用 UART3。保留屏幕和视觉，便于现场诊断。
            uart = None
            sender = None
            print("UART disabled:", error)

        start_camera(sensor)
        detector = LineDetector()
        print("Camera CH1 grayscale vision initialized")

        if display_ready:
            try:
                # 底图由 CH0 硬件直通 VIDEO1；这两个透明缓冲区只画调试 OSD。
                display_buffers = (
                    image_module.Image(
                        cfg.DISPLAY_WIDTH,
                        cfg.DISPLAY_HEIGHT,
                        image_module.ARGB8888,
                    ),
                    image_module.Image(
                        cfg.DISPLAY_WIDTH,
                        cfg.DISPLAY_HEIGHT,
                        image_module.ARGB8888,
                    ),
                )
                display_buffers[0].clear()
                display_buffers[1].clear()
                print("Display OSD double buffer initialized")
            except Exception as error:
                print("Display buffer allocation failed:", error)
                display_ready = False
                try:
                    Display.deinit()
                except Exception:
                    pass

        now = time.ticks_ms()
        next_send = now
        next_display = now
        last_diag = now

        while True:
            image = sensor.snapshot(chn=CAM_CHN_ID_1)
            result = detector.process(image)
            frame_count += 1
            diag_frame_count += 1
            now = time.ticks_ms()

            if time.ticks_diff(now, next_send) >= 0:
                if sender is not None:
                    sender.send(result)
                next_send = _ticks_add(next_send, cfg.SEND_PERIOD_MS)
                # 处理超时后不补发旧帧，直接从当前时刻重新排期。
                if time.ticks_diff(now, next_send) >= 0:
                    next_send = _ticks_add(now, cfg.SEND_PERIOD_MS)

            if time.ticks_diff(now, last_diag) >= cfg.DEBUG_PRINT_PERIOD_MS:
                elapsed = max(1, time.ticks_diff(now, last_diag))
                measured_fps = diag_frame_count * 1000.0 / elapsed
                if cfg.DEBUG_PRINT:
                    print(
                        "fps=%.1f th=%d strips=%d valid=%d conf=%d "
                        "lat=%d head=%d curv=%d txerr=%d"
                        % (
                            measured_fps,
                            result.threshold,
                            result.raw_valid_strips,
                            result.valid,
                            result.confidence,
                            result.lateral_error_mm,
                            result.heading_error_deg,
                            result.curvature,
                            sender.tx_error if sender is not None else -1,
                        )
                    )
                diag_frame_count = 0
                last_diag = now

            if display_ready and time.ticks_diff(now, next_display) >= 0:
                try:
                    display_image = display_buffers[display_buffer_index]
                    display_image.clear()
                    scale_x = cfg.DISPLAY_WIDTH / cfg.IMAGE_WIDTH
                    scale_y = cfg.DISPLAY_HEIGHT / cfg.IMAGE_HEIGHT
                    detector.draw_debug_overlay(
                        display_image,
                        result,
                        measured_fps,
                        sender.tx_error if sender is not None else -1,
                        scale_x=scale_x,
                        scale_y=scale_y,
                    )
                    Display.show_image(
                        display_image,
                        0,
                        0,
                        layer=Display.LAYER_OSD0,
                    )
                    display_buffer_index ^= 1
                except Exception as error:
                    print("Display stopped:", error)
                    display_ready = False
                    try:
                        Display.deinit()
                    except Exception:
                        pass
                next_display = _ticks_add(now, cfg.DISPLAY_PERIOD_MS)

            if frame_count % cfg.GC_PERIOD_FRAMES == 0:
                gc.collect()

            try:
                os.exitpoint()
            except AttributeError:
                pass

    except KeyboardInterrupt:
        pass
    except Exception as error:
        if sender is not None:
            sender.send_invalid(255)
        print("K230 vision stopped:", error)
        raise
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass
        display_buffers = None
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    run()
