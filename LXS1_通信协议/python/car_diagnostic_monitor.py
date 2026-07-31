"""小车USART1/ECB02诊断帧监视器。

用法：
    pip install pyserial
    python car_diagnostic_monitor.py --list
    python car_diagnostic_monitor.py --port COM7
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import struct
import sys
import time

from lxs1 import MSG_CAR_DIAGNOSTIC, Parser


PAYLOAD = struct.Struct("<BBBBHiiiiiihhiiHHHI")
MISSION_STATES = {
    0: "READY",
    1: "STARTING",
    2: "NORMAL_TRACK",
    3: "ACTION_SLOW",
    4: "LOST_HOLD",
    5: "REACQUIRE",
    6: "FINISH_CONFIRM",
    7: "DONE",
    8: "ABORT",
    9: "FAULT",
}
LINE_STATES = {
    0: "DISABLED",
    1: "TRACKING",
    2: "LOST_HOLD",
    3: "REACQUIRE",
    4: "STOPPED",
}


@dataclass(slots=True)
class Diagnostic:
    run_id: int
    mission_state: int
    line_state: int
    flags: int
    fault: int
    left_requested_pps: int
    right_requested_pps: int
    left_target_pps: int
    right_target_pps: int
    left_speed_pps: int
    right_speed_pps: int
    left_pwm_permille: int
    right_pwm_permille: int
    left_counts: int
    right_counts: int
    vision_age_ms: int
    vision_frames: int
    vision_malformed: int
    path_mm: int

    def flag(self, bit: int) -> bool:
        return bool(self.flags & (1 << bit))


def decode_diagnostic(data: bytes) -> Diagnostic:
    if len(data) != PAYLOAD.size:
        raise ValueError(f"CAR_DIAGNOSTIC长度应为{PAYLOAD.size}，实际{len(data)}")
    return Diagnostic(*PAYLOAD.unpack(data))


def diagnoses(d: Diagnostic) -> list[str]:
    result: list[str] = []
    key_raw = d.flag(0)
    key_debounced = d.flag(1)
    vision_received = d.flag(2)
    vision_valid = d.flag(3)
    standby = d.flag(5)
    direction_forward = d.flag(6)
    start_delay = d.flag(7)
    commanded = max(d.left_requested_pps, d.right_requested_pps) > 0
    ramping = max(d.left_target_pps, d.right_target_pps) > 0
    pwm_active = max(d.left_pwm_permille, d.right_pwm_permille) > 20
    moving = max(abs(d.left_speed_pps), abs(d.right_speed_pps)) > 100

    if d.fault:
        names = []
        if d.fault & 0x01:
            names.append("左编码器方向反")
        if d.fault & 0x02:
            names.append("右编码器方向反")
        result.append("控制器FAULT=0x%04X：%s" %
                      (d.fault, "、".join(names) or "未知故障位"))

    if d.mission_state == 0:
        if start_delay:
            result.append("按键已锁存，正在执行10 s安全撤离倒计时")
        elif not key_raw:
            result.append("开始键未到达PC0：检查按键板K3→H3-7/PC0、公共GND和内部上拉配置")
        elif not key_debounced:
            result.append("PC0已拉低但尚未消抖：持续按住K3至少20 ms")
        else:
            result.append("按键已识别但状态仍为READY：检查任务主循环是否运行")
    elif not key_raw:
        result.append("按键已经触发任务，当前松开是正常现象")

    if d.mission_state == 1:
        if not commanded:
            result.append("STARTING但目标为0：启动直行命令没有送到速度环")
        elif start_delay and not ramping:
            result.append("目标已下发，正在等待上电2 s电机保护延时")
        elif ramping and not pwm_active:
            result.append("速度目标已进入斜坡但PWM仍为0：检查速度控制任务/故障")
        elif pwm_active and not standby:
            result.append("PWM非零但STBY为低：检查PB14逻辑或TB6612 STBY接线")
        elif pwm_active and standby and not moving:
            result.append(
                "软件已输出PWM且STBY为高，但编码器速度为0："
                "若轮子也不转，检查TB6612的VM/VCC/GND、PWMA/PWMB、方向线和电机输出；"
                "若轮子实际在转，检查E1/E2编码器接线"
            )

    if commanded and standby and not direction_forward:
        result.append("STBY已使能但方向脚读回不是前进组合：检查PB0/PB1/PB12/PB13")

    if pwm_active and standby:
        if abs(d.left_speed_pps) <= 100 < abs(d.right_speed_pps):
            result.append("仅左轮无编码器响应：检查E1A→PB6、E1B→PB7及左电机通道")
        if abs(d.right_speed_pps) <= 100 < abs(d.left_speed_pps):
            result.append("仅右轮无编码器响应：检查E2A→PA5、E2B→PA1及右电机通道")

    if not vision_received:
        result.append("未收到K230合法帧：检查GPIO32→PB11、共地及19字节新协议")
    elif d.vision_malformed:
        result.append(
            f"K230格式错误累计{d.vision_malformed}：确认K230和STM32同时更新到19字节协议"
        )
    elif d.mission_state == 1 and not vision_valid:
        result.append(
            f"K230链路在线但暂无线有效（帧龄{d.vision_age_ms} ms），小车应保持低速直行搜索"
        )

    if commanded and pwm_active and standby and moving and d.fault == 0:
        result.append("电机驱动和编码器均已有响应")
    return result


def format_status(d: Diagnostic) -> str:
    mission = MISSION_STATES.get(d.mission_state, str(d.mission_state))
    line = LINE_STATES.get(d.line_state, str(d.line_state))
    return (
        f"run={d.run_id} mission={mission} line={line} "
        f"KEY(raw/deb)={int(d.flag(0))}/{int(d.flag(1))} "
        f"REQ={d.left_requested_pps}/{d.right_requested_pps} "
        f"TGT={d.left_target_pps}/{d.right_target_pps} "
        f"SPD={d.left_speed_pps}/{d.right_speed_pps} "
        f"PWM={d.left_pwm_permille}/{d.right_pwm_permille} "
        f"STBY={int(d.flag(5))} DIR={int(d.flag(6))} "
        f"K230(age/ok/bad)={d.vision_age_ms}/{d.vision_frames}/{d.vision_malformed} "
        f"fault=0x{d.fault:04X}"
    )


def list_ports() -> int:
    try:
        from serial.tools import list_ports as serial_list_ports
    except ImportError:
        print("缺少pyserial，请执行：pip install pyserial", file=sys.stderr)
        return 2
    ports = list(serial_list_ports.comports())
    if not ports:
        print("未发现串口")
        return 1
    for port in ports:
        print(f"{port.device}: {port.description}")
    return 0


def monitor(port: str, baud: int, once: bool) -> int:
    try:
        import serial
    except ImportError:
        print("缺少pyserial，请执行：pip install pyserial", file=sys.stderr)
        return 2

    parser = Parser()
    last_rx = time.monotonic()
    last_print = 0.0
    print(f"监听 {port} @ {baud}，等待MSG 0x34；按Ctrl+C退出")
    try:
        with serial.Serial(port, baud, timeout=0.2) as uart:
            while True:
                chunk = uart.read(256)
                now = time.monotonic()
                for frame in parser.feed(chunk):
                    if frame.msg_id != MSG_CAR_DIAGNOSTIC:
                        continue
                    last_rx = now
                    if (now - last_print) < 0.45 and not once:
                        continue
                    diagnostic = decode_diagnostic(frame.data)
                    print("\n" + format_status(diagnostic))
                    for item in diagnoses(diagnostic):
                        print("  - " + item)
                    last_print = now
                    if once:
                        return 0
                if now - last_rx > 2.0:
                    print(
                        "超过2 s未收到诊断帧：检查USART1 PA9(TX)→接收端RX、"
                        "115200 8N1、ECB02链路和共地",
                        file=sys.stderr,
                    )
                    last_rx = now
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"串口错误：{error}", file=sys.stderr)
        return 1


def main() -> int:
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("--list", action="store_true", help="列出串口")
    argument_parser.add_argument("--port", help="例如COM7")
    argument_parser.add_argument("--baud", type=int, default=115200)
    argument_parser.add_argument("--once", action="store_true", help="收到一帧后退出")
    args = argument_parser.parse_args()
    if args.list:
        return list_ports()
    if not args.port:
        argument_parser.error("请指定--port，或先使用--list")
    return monitor(args.port, args.baud, args.once)


if __name__ == "__main__":
    raise SystemExit(main())
