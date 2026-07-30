"""ST7701 独立显示测试。

本程序不初始化摄像头和 UART。先用它确认屏幕、排线和固件是否正常。
"""

import gc
import os
import time

import image
from media.display import Display
from media.media import MediaManager

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480


def run():
    screen = None
    try:
        Display.init(
            Display.ST7701,
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            to_ide=True,
        )
        MediaManager.init()

        screen = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
        screen.clear()

        bar_width = DISPLAY_WIDTH // 6
        colors = (
            (255, 0, 0),
            (255, 255, 0),
            (0, 255, 0),
            (0, 255, 255),
            (0, 0, 255),
            (255, 0, 255),
        )
        for index in range(6):
            screen.draw_rectangle(
                index * bar_width,
                0,
                bar_width + 1,
                DISPLAY_HEIGHT,
                color=colors[index],
                fill=True,
            )

        screen.draw_rectangle(
            70, 180, 500, 120, color=(0, 0, 0), fill=True
        )
        screen.draw_string_advanced(
            105,
            195,
            42,
            "ST7701 DISPLAY OK",
            color=(255, 255, 255),
        )
        screen.draw_string_advanced(
            125,
            250,
            28,
            "640 x 480  /  YAHBOOM K230",
            color=(255, 255, 255),
        )
        Display.show_image(screen)
        print("ST7701 test initialized: 640x480")

        while True:
            time.sleep_ms(100)
            try:
                os.exitpoint()
            except AttributeError:
                pass

    except KeyboardInterrupt:
        pass
    except Exception as error:
        print("ST7701 test failed:", error)
        raise
    finally:
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        screen = None
        gc.collect()


if __name__ == "__main__":
    run()
