# LXS1 AI 工作参考

## 不可改变的协议常量

- SOF：`AA 55`
- EOF：`0D 0A`
- `LEN = 3 + DATA长度`
- 小端序
- 最大数据：64字节
- 帧：`SOF | LEN | SRC | DST | MSG | DATA | EOF`
- 不使用版本、序号、时间戳、校验位、ACK、重试或CRC字段。

节点：`0x01`凌霄适配端，`0x02`飞机F407，`0x03`天空K230，`0x04`小车F407，`0x05`小车K230，`0x06`地面站，`0xFF`广播。

## 设备职责

| 设备 | 必须负责 | 不要让它负责 |
|---|---|---|
| 凌霄适配端 | 把语义飞行指令转换为原厂帧，回传飞控状态 | 不要假设原厂协议等同于LXS1 |
| 飞机F407 | 任务状态机、视觉结果融合、飞行指令、抛投/动态降落时序 | 不要把任务状态机放在K230 |
| 天空K230 | 识别小车、平台和目标，输出误差/置信度/有效期 | 不要输出电机控制量或重复旧目标 |
| 小车F407 | 电机、循线控制、启动、速度、路径进度 | 不要由地面站实时手动遥控 |
| 小车K230 | 黑线检测，输出横向误差/航向误差/曲率/丢线计数 | 不要修改小车任务状态 |
| 地面站 | 监视、记录、查询、终止、故障显示 | 不要使用图传协议或测评时改代码 |

## 消息号

```text
01 HELLO             02 HEARTBEAT        03 ACK       04 NACK
10 TASK_START        11 TASK_ABORT       12 TASK_STATE 13 RUN_RESULT
20 FC_CMD            21 FC_STATE         22 FC_POSE
30 CAR_CMD           31 CAR_STATE        32 CAR_POSE
40 VISION_TARGET     41 VISION_LANDMARK  42 VISION_LINE 43 VISION_DIAG
50 DROP_STATE        51 LAND_STATE       60 FAULT
```

核心负载：

```text
TASK_START:      task_mode:u8, run_id:u8, start_nonce:u16, car_speed_mm_s:u16
VISION_TARGET:   valid:u8, target_kind:u8, confidence:u16, dx/dy/dz:i16,
                 yaw:i16, age_ms:u16, flags:u16, reserved:u16
VISION_LANDMARK: valid:u8, marker_kind:u8, confidence:u16,
                 error_x/error_y/error_yaw:i16, scale:u16, flags:u16, reserved:u16
VISION_LINE:     valid:u8, lost_count:u8, confidence:u16,
                 lateral_error:i16, heading_error:i16, curvature:i16, reserved:u16
```

飞机任务状态：`IDLE → TAKEOFF → HOVER_3S → SEARCH → FOLLOW → DROP/LAND_CAR → WAIT_5S → RETURN → LAND_H → DONE`；异常进入 `ABORT`。

## 时序与安全

- 视觉目标超过300 ms无更新，置无效；不要无限保持旧目标。
- 飞控状态/指令超过300 ms无更新，进入悬停或安全返航。
- 小车视觉超过100 ms无更新，保持短时控制；超过500 ms关闭电机并报错。
- 心跳1 Hz；视觉最高20 Hz；飞行状态20 Hz；小车状态10 Hz。
- 副作用命令使用 `run_id` 幂等；不要通过重试重复触发起飞、抛投或降落。
- STM32使用DMA+IDLE接收，ISR只做搬运/置标志；主循环做解析和分发；禁止动态内存、阻塞、延时和中断内打印。

## 现有工程落地位置

- 规范：`LXS1_通信协议/LXS1_PROTOCOL.md`
- STM32头文件：`LXS1_通信协议/include/lxs1_protocol.h`
- STM32实现：`LXS1_通信协议/src/lxs1_protocol.c`
- Python/K230参考：`LXS1_通信协议/python/lxs1.py`
- 测试：`LXS1_通信协议/tests/test_lxs1.py`

原 KGS1/RTSP 图传和 `AA FF F1` 旧检测串口格式不属于本协议。
