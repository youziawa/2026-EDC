"""LXS1 小车视觉串口发送器。

帧格式：
AA 55 | LEN | SRC | DST | MSG | DATA | 0D 0A

VISION_LINE 的 DATA 固定为 11 字节，不含校验和、CRC 或保留字段。
"""

FRAME_HEAD_0 = 0xAA
FRAME_HEAD_1 = 0x55
FRAME_TAIL_0 = 0x0D
FRAME_TAIL_1 = 0x0A

MSG_VISION_LINE = 0x42
VISION_DATA_LENGTH = 11
VISION_FRAME_LENGTH = 19


def _clamp_int(value, minimum, maximum):
    value = int(value)
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _put_u16_le(buffer, index, value):
    value = int(value) & 0xFFFF
    buffer[index] = value & 0xFF
    buffer[index + 1] = (value >> 8) & 0xFF


class VisionLineSender:
    """复用同一个 bytearray，避免每帧打包产生临时对象。"""

    def __init__(self, uart, src=0x05, dst=0x04):
        self.uart = uart
        self.frame = bytearray(VISION_FRAME_LENGTH)
        self.frame[0] = FRAME_HEAD_0
        self.frame[1] = FRAME_HEAD_1
        self.frame[2] = 3 + VISION_DATA_LENGTH
        self.frame[3] = src & 0xFF
        self.frame[4] = dst & 0xFF
        self.frame[5] = MSG_VISION_LINE
        self.frame[17] = FRAME_TAIL_0
        self.frame[18] = FRAME_TAIL_1

        self.tx_ok = 0
        self.tx_error = 0

    def update(self, valid, lost_count, confidence,
               lateral_error_mm, heading_error_deg, curvature,
               marker_detected=0):
        frame = self.frame
        frame[6] = 1 if valid else 0
        frame[7] = _clamp_int(lost_count, 0, 255)

        _put_u16_le(frame, 8, _clamp_int(confidence, 0, 1000))
        _put_u16_le(frame, 10, _clamp_int(lateral_error_mm, -32768, 32767))
        _put_u16_le(frame, 12, _clamp_int(heading_error_deg, -32768, 32767))
        _put_u16_le(frame, 14, _clamp_int(curvature, -32768, 32767))
        frame[16] = 1 if marker_detected else 0
        return frame

    def send(self, result):
        self.update(result.valid,
                    result.lost_count,
                    result.confidence,
                    result.lateral_error_mm,
                    result.heading_error_deg,
                    result.curvature,
                    result.marker_detected)
        try:
            written = self.uart.write(self.frame)
            if written == VISION_FRAME_LENGTH:
                self.tx_ok += 1
                return True
        except Exception:
            pass

        self.tx_error += 1
        return False

    def send_invalid(self, lost_count=255):
        self.update(0, lost_count, 0, 0, 0, 0, 0)
        try:
            written = self.uart.write(self.frame)
            if written == VISION_FRAME_LENGTH:
                self.tx_ok += 1
                return True
        except Exception:
            pass

        self.tx_error += 1
        return False
