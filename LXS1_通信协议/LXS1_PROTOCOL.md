# LXS1 陆空协同简易通信协议

版本：`1.0`  
适用设备：凌霄飞控适配端、飞机主控 STM32F407VET6、天空端 K230、小车主控 STM32F407VET6、小车视觉 K230、地面站。

本协议不使用校验位、不使用 CRC，只规定帧头、长度、地址、消息号、数据和帧尾。

## 1. 总体分工

```text
飞机主控 F407  ── UART ── 天空端 K230
       │
       └── 凌霄飞控适配接口 ── 凌霄飞控

小车主控 F407  ── UART ── 小车视觉 K230

飞机主控、小车主控、地面站之间：透明无线串口
```

- 飞机主控负责任务状态机、飞行动作和抛投/动态降落时序。
- 小车主控负责电机、循线、启动、停车和小车状态。
- 天空端 K230 只输出小车/平台视觉结果。
- 小车 K230 只输出黑线识别结果。
- 地面站只显示、记录、查询和发送终止指令。
- 凌霄飞控的原厂协议不属于 LXS1，必须由飞机主控中的适配层转换。
- 不使用原来的 KGS1、RTSP 和 `AA FF F1` 图传/检测协议。

## 2. 物理链路

默认所有串口使用：`115200、8数据位、无校验、1停止位`。

| 链路 | 连接 | 内容 |
|---|---|---|
| 飞机主控-K230 | UART | 小车目标、平台中心、视觉状态 |
| 小车主控-K230 | UART | 黑线横向误差、航向误差、丢线状态 |
| 飞机主控-凌霄 | 原厂 UART/CAN，由 Adapter转换 | 起飞、定高、飞行、降落和飞控状态 |
| 飞机主控-地面站 | 透明无线串口 | 飞行状态和任务遥测 |
| 小车主控-地面站 | 透明无线串口 | 小车状态和任务遥测 |

若无线模块只支持点对点，则使用透明转发方式，不改变原帧内容。

## 3. 帧格式

```text
+--------+--------+------+-----+-----+-----+----------+--------+
| 帧头1  | 帧头2  | LEN  | SRC | DST | MSG | DATA     | 帧尾   |
|  0xAA  |  0x55  | 1字节| 1字节|1字节|1字节| 0~64字节 |0D 0A  |
+--------+--------+------+-----+-----+-----+----------+--------+
```

完整字节顺序：

```text
AA 55 | LEN | SRC | DST | MSG | DATA[0] ... DATA[N-1] | 0D 0A
```

规定：

- `LEN = 3 + DATA长度`，统计 `SRC、DST、MSG、DATA`。
- `DATA` 最大64字节。
- 总帧长度最大72字节：`2 + 1 + 3 + 64 + 2`。
- 多字节数据统一小端序。
- 固定帧尾位于长度字段指定的位置，不在数据中搜索帧尾。
- 数据中如果出现 `AA、55、0D、0A`，仍按 `LEN`解析，不需要转义。
- 收到错误长度或错误帧尾时，丢弃当前帧并重新搜索 `AA 55`。

## 4. 设备地址

| 地址 | 设备 |
|---:|---|
| `0x01` | 凌霄飞控适配端 |
| `0x02` | 飞机主控 STM32F407VET6 |
| `0x03` | 天空端 K230 |
| `0x04` | 小车主控 STM32F407VET6 |
| `0x05` | 小车视觉 K230 |
| `0x06` | 地面站 |
| `0xFF` | 广播 |

## 5. 消息号

| MSG | 名称 | 发送方 | 接收方 |
|---:|---|---|---|
| `0x01` | HELLO | 所有设备 | 地面站/对应主控 |
| `0x02` | HEARTBEAT | 所有设备 | 地面站/对应主控 |
| `0x10` | TASK_START | 小车主控 | 广播 |
| `0x11` | TASK_ABORT | 地面站/飞机主控 | 广播 |
| `0x12` | TASK_STATE | 飞机主控 | 地面站/小车主控 |
| `0x20` | FC_CMD | 飞机主控 | 凌霄适配端 |
| `0x21` | FC_STATE | 凌霄适配端 | 飞机主控/地面站 |
| `0x22` | FC_POSE | 凌霄适配端 | 飞机主控/地面站 |
| `0x30` | CAR_CMD | 飞机主控/地面站 | 小车主控 |
| `0x31` | CAR_STATE | 小车主控 | 飞机主控/地面站 |
| `0x32` | CAR_POSE | 小车主控 | 飞机主控/地面站 |
| `0x40` | VISION_TARGET | 天空端K230 | 飞机主控 |
| `0x41` | VISION_LANDMARK | 天空端K230 | 飞机主控 |
| `0x42` | VISION_LINE | 小车K230 | 小车主控 |
| `0x43` | VISION_DIAG | K230 | 对应主控/地面站 |
| `0x50` | DROP_STATE | 飞机主控 | 地面站 |
| `0x51` | LAND_STATE | 飞机主控 | 地面站 |
| `0x60` | FAULT | 所有设备 | 地面站/对应主控 |

## 6. 数据定义

### 6.1 TASK_START

```text
DATA[0]  task_mode
DATA[1]  run_id
DATA[2:3]  car_speed_mm_s，小端
```

`task_mode=1`为抛投任务，`task_mode=2`为动态起降任务。`run_id`每次测试递增，重复收到相同 `run_id`时不能重复启动电机或重复起飞。

### 6.2 TASK_STATE

```text
DATA[0]  task_mode
DATA[1]  state
DATA[2]  fault
DATA[3:6] elapsed_ms，小端，3字节
DATA[6:9] car_path_mm，小端，3字节
```

飞机任务状态：

```text
0 IDLE       1 TAKEOFF       2 HOVER_3S
3 SEARCH     4 FOLLOW        5 DROP
6 LAND_CAR   7 WAIT_5S       8 RETURN
9 LAND_H     10 DONE         11 ABORT
```

### 6.3 FC_CMD

```text
DATA[0]  mode
DATA[1]  command
DATA[2:3] target_height_cm，小端
DATA[4:5] target_vx_cm_s，小端，有符号
DATA[6:7] target_vy_cm_s，小端，有符号
DATA[8:9] target_vz_cm_s，小端，有符号
```

`command`示例：`0待机`、`1起飞`、`2悬停`、`3跟踪`、`4下降`、`5返航`、`6降落`、`7中止`。

### 6.4 FC_STATE

```text
DATA[0]  state
DATA[1]  armed
DATA[2:3] height_cm，小端
DATA[4:5] vx_cm_s，有符号
DATA[6:7] vy_cm_s，有符号
DATA[8:9] vz_cm_s，有符号
DATA[10:11] battery_mv，小端
DATA[12] fault
```

### 6.5 CAR_STATE

```text
DATA[0]  state
DATA[1]  speed_cm_s
DATA[2:4] path_cm，小端
DATA[4]  lap
DATA[5]  line_valid
DATA[6]  fault
```

小车状态：`0待机`、`1运行`、`2完成`、`3故障`、`4中止`。

### 6.6 CAR_POSE

```text
DATA[0:1] x_cm，小端
DATA[2:3] y_cm，小端
DATA[4:5] heading_deg，小端，有符号
DATA[6:7] path_cm，小端
```

### 6.7 VISION_TARGET

```text
DATA[0]  valid
DATA[1]  target_kind：1小车，2平台
DATA[2:3] confidence，0~1000
DATA[4:5] dx_cm，有符号
DATA[6:7] dy_cm，有符号
DATA[8:9] dz_cm，有符号
DATA[10:11] yaw_deg，有符号
```

### 6.8 VISION_LANDMARK

```text
DATA[0]  valid
DATA[1]  marker_kind
DATA[2:3] confidence，0~1000
DATA[4:5] error_x_cm，有符号
DATA[6:7] error_y_cm，有符号
DATA[8:9] error_yaw_deg，有符号
```

### 6.9 VISION_LINE

```text
DATA[0]  valid
DATA[1]  lost_count
DATA[2:3] confidence，0~1000
DATA[4:5] lateral_error_mm，有符号
DATA[6:7] heading_error_deg，有符号
DATA[8:9] curvature，有符号
```

### 6.10 DROP_STATE、LAND_STATE、FAULT

```text
DROP_STATE:
DATA[0] state：0未动作，1释放，2完成，3失败
DATA[1] object_mass_g

LAND_STATE:
DATA[0] state：0未开始，1下降，2已着陆，3停留，4完成，5失败
DATA[1:2] stable_time_s，小端

FAULT:
DATA[0] source
DATA[1] fault_code
DATA[2] severity：1提示，2警告，3错误
```

## 7. 发送周期和安全策略

| 消息 | 周期 |
|---|---:|
| VISION_TARGET、VISION_LANDMARK | 10~20 Hz |
| VISION_LINE | 20~50 Hz |
| FC_STATE | 20 Hz |
| CAR_STATE、CAR_POSE | 10 Hz |
| HEARTBEAT | 1 Hz |
| TASK_STATE | 5~10 Hz |

- 视觉数据超过300 ms未更新，主控将 `valid=0`，不得无限使用旧数据。
- 小车视觉超过500 ms未更新，小车停止电机并上报故障。
- 飞控状态超过500 ms未更新，飞机主控停止发送运动指令并进入安全状态。
- 地面站超过2秒未收到遥测，只显示离线，不改变当前任务。
- 不设置ACK和重试机制；需要防重复的启动动作使用 `run_id`，动作命令由任务状态机单次触发。

## 8. STM32接收要求

- UART使用DMA+空闲中断接收。
- 中断中只记录接收长度或搬运数据，不做完整业务解析。
- 主循环按 `AA 55` 搜索帧头，读取 `LEN`，等待完整帧，再检查固定帧尾 `0D 0A`。
- 校验通过帧尾后，根据 `SRC、DST、MSG`分发。
- 所有缓冲区静态分配，不使用动态内存。
