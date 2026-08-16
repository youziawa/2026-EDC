# LXS1 陆空协同通信协议

版本：`1.1`
字节序：多字节整数统一小端序
默认串口：`115200 8N1`

本文件与 `../docs/architecture/mission-state-machine.yaml`、小车 STM32 当前实现保持一致。凌霄飞控
原厂协议不属于 LXS1，由飞机主控适配层转换。

> **协议修订（2026-07-31）**：`0x40` 是天空端 K230 到飞机 F4 的**机载本地像素
> 测量帧**，不是节点间厘米坐标遥测。K230 只输出像素测量；F4 结合飞控实际高度
> 完成距离计算并用于控制。该帧不得转发到小车、地面站或凌霄飞控。原有帧头、
> 长度、编号、字节序和 K230 字段均不变。

## 1. 物理拓扑

ECB02不能一主多从，系统使用三条独立点对点链路、每个设备两块无线模块：

```text
飞机主控 USART? ←→ ECB02 ←→ ECB02 ←→ 小车 USART2
飞机主控 USART? ←→ ECB02 ←→ ECB02 ←→ 地面站串口
小车 USART1     ←→ ECB02 ←→ ECB02 ←→ 地面站串口

飞机主控 UART   ←→ 天空端K230
小车 USART3     ←→ 小车K230
```

小车端已固定：

| UART | 对端 |
|---|---|
| USART1 | 地面站无线链路 |
| USART2 | 飞机主控无线链路 |
| USART3 | 小车K230，PB10 TX、PB11 RX |

## 2. 帧格式

```text
AA 55 | LEN | SRC | DST | MSG | DATA[0..N-1] | 0D 0A
```

- `LEN = 3 + N`，包含 `SRC、DST、MSG、DATA`。
- `N`为0～64，总帧最大72字节。
- 按LEN确定帧尾位置，不在DATA中搜索帧尾，不转义。
- 长度或帧尾错误时丢弃并重新搜索 `AA 55`。
- 当前协议无CRC、无ACK；任务副作用以 `run_id` 幂等。

## 3. 节点地址

| 地址 | 节点 |
|---:|---|
| `0x01` | 凌霄飞控适配端 |
| `0x02` | 飞机主控 |
| `0x03` | 天空端K230 |
| `0x04` | 小车主控 |
| `0x05` | 小车K230 |
| `0x06` | 地面站 |
| `0xFF` | 广播 |

## 4. 消息号

| MSG | 名称 | 主要方向 |
|---:|---|---|
| `0x01` | HELLO | 任意→对应主控/地面站 |
| `0x02` | HEARTBEAT | 任意→对应主控/地面站 |
| `0x10` | TASK_START | 小车→飞机、地面站 |
| `0x11` | TASK_ABORT | 地面站或小车实体复位键→飞机、小车、地面站 |
| `0x12` | TASK_STATE | 飞机→小车、地面站 |
| `0x20` | FC_CMD | 飞机主控→飞控适配端 |
| `0x21` | FC_STATE | 飞控适配端→飞机、地面站 |
| `0x22` | FC_POSE | 飞控适配端→飞机、地面站 |
| `0x30` | CAR_CMD | 预留；飞机/地面站→小车 |
| `0x31` | CAR_STATE | 小车→飞机、地面站 |
| `0x32` | CAR_POSE | 小车→飞机、地面站 |
| `0x33` | TRACK_EVENT | 小车→飞机、地面站 |
| `0x34` | CAR_DIAGNOSTIC | 小车→地面站 |
| `0x40` | VISION_PIXEL | 天空端K230→飞机F4（机载本地） |
| `0x41` | VISION_LANDMARK | 预留（当前天空K230未定义发送格式） |
| `0x42` | VISION_LINE | 小车K230→小车 |
| `0x43` | VISION_DIAG | K230→对应主控/地面站 |
| `0x50` | DROP_STATE | 飞机→地面站 |
| `0x51` | LAND_STATE | 飞机→地面站 |
| `0x60` | FAULT | 任意→对应主控/地面站 |

## 5. 公共任务定义

任务模式：

```text
1 DROP
2 DYNAMIC_LANDING
```

全局状态：

```text
0 IDLE       1 TAKEOFF     2 HOVER_3S   3 SEARCH
4 FOLLOW     5 DROP        6 LAND_CAR   7 WAIT_5S
8 RETAKEOFF  9 RETURN     10 LAND_H    11 DONE
12 ABORT    13 FAULT
```

赛道事件：

```text
1 B_CROSS
2 C_ENTER
3 D_EXIT
4 A_FINISH
```

小车状态：

```text
0 READY        1 STARTING      2 NORMAL_TRACK
3 ACTION_SLOW  4 LOST_HOLD     5 REACQUIRE
6 FINISH_CONFIRM  7 DONE       8 ABORT
9 FAULT
```

## 6. 任务消息载荷

### 6.1 TASK_START `0x10`，6字节

小车任务真正开始（10秒安全倒计时结束）时发送到飞机和地面站。

```text
0      run_id:u8
1      task_mode:u8
2..3   normal_speed_mm_s:u16
4..5   action_speed_mm_s:u16
```

飞机只对新 `run_id`执行一次起飞和任务初始化；重复帧不得重复产生副作用。

### 6.2 TASK_ABORT `0x11`，至少2字节

```text
0      run_id:u8
1      reason:u8
```

仅当 `run_id`等于当前任务时执行。飞机安全降落，小车立即停车。除地面站紧急中止外，
小车1×4按键板的K4也可由小车节点发出该帧，接收端必须接受来源为
`LXS1_NODE_CAR_MCU` 的实体安全复位。

### 6.3 TASK_STATE `0x12`，14字节，飞机以100 ms周期发送

```text
0      run_id:u8
1      task_mode:u8
2      global_state:u8
3      result:u8
4..5   fault_code:u16
6..9   elapsed_ms:u32
10..13 car_path_mm:u32
```

小车当前至少处理 `run_id`和 `global_state`；收到 `RETURN=9`时可退出动作慢速。

### 6.4 TRACK_EVENT `0x33`，6字节

```text
0      run_id:u8
1      event:u8
2..5   path_mm:u32
```

地图里程触发点：B=1500、C=3856、D=5356、A=7712 mm。`C_ENTER`在C—D
动作区每100 ms重发，不能依赖单帧；B、D、A为单次事件。

## 7. 小车遥测

### 7.1 CAR_STATE `0x31`，13字节，100 ms

```text
0      run_id:u8
1      car_state:u8
2      speed_mode:u8       0停止，1正常，2动作慢速，3搜索/恢复
3      line_valid:u8
4..5   speed_mm_s:u16
6..7   fault_code:u16
8..11  path_mm:u32
12     lap:u8
```

### 7.2 CAR_POSE `0x32`，11字节，100 ms

```text
0      run_id:u8
1..2   x_mm:i16
3..4   y_mm:i16
5..6   heading_deg:i16
7..10  path_mm:u32
```

### 7.3 CAR_DIAGNOSTIC `0x34`，52字节，100 ms

仅发送到地面站：

```text
0 run_id:u8                 1 car_state:u8
2 line_state:u8             3 flags:u8
4..5 fault_code:u16         6..9 left_requested_pps:i32
10..13 right_requested_pps:i32
14..17 left_target_pps:i32  18..21 right_target_pps:i32
22..25 left_speed_pps:i32   26..29 right_speed_pps:i32
30..31 left_pwm_permille:i16
32..33 right_pwm_permille:i16
34..37 left_total_counts:i32
38..41 right_total_counts:i32
42..43 k230_age_ms:u16      44..45 k230_frame_count_low16:u16
46..47 malformed_low16:u16  48..51 path_mm:u32
```

`flags` bit0～7：

```text
key_raw, key_debounced, k230_received, k230_fresh_valid,
marker_fresh, motor_standby, direction_forward, motor_start_delay
```

## 8. 天空视觉本地接口

本节只适用于同一架飞机内的“天空端 K230 ↔ 飞机 F4”串口。该接口复用 LXS1
帧外壳以复用解析器，但其载荷是像素测量，**不属于飞机—小车—地面站之间的
厘米坐标协议**。

F4 收到有效像素帧后，必须使用同一时刻的新鲜飞控实际高度（及后续需要时的姿态）
计算目标相对位置；F4 内部统一得到 FRD：X 前、Y 右、Z 下，距离单位 cm。若
控制器内部使用“Y 左为正”等坐标，只允许在 F4 控制适配层转换。

### 8.1 VISION_PIXEL `0x40`，12字节，10～20 Hz

```text
0      valid:u8
1      target_kind:u8      1小车，2小车起降平台
2..3   confidence:u16      0..1000
4..5   dx_px:i16           平台中心相对图像中心的水平像素偏差，右方为正
6..7   dy_px:i16           平台中心相对图像中心的垂直像素偏差，下方为正
8..9   radius_px:i16       外圆半径，像素
10..11 vision_mode:i16     1 ACQUIRE，2 TRACK，3 LOST
```

`target_kind`：`1` 小车，`2` 小车起降平台。现有脚本固定发送 `2`。

`VISION_LANDMARK 0x41` 保留编号但当前未定义载荷，禁止根据旧版厘米字段实现或发送。
后续如需 K230 辅助 H 点、投放标志或平台精对准，应先补充同样由 F4 完成尺度换算的
本地像素载荷定义，再同步更新本文件、C/Python 库和各端实现。

要求：

- `valid=0`时 `dx_px/dy_px/radius_px`必须清零，不沿用旧结果；`vision_mode`允许为
  `VISION_LOST`。
- 飞机F4按接收时刻判断新鲜度；超过300 ms按视觉无效处理。控制用的新鲜度可更严，
  例如120 ms，但不得用1 Hz心跳刷新视觉时间戳。
- SEARCH阶段允许 `valid=0`；FOLLOW/DROP/LAND_CAR进入精确动作前必须满足连续
  有效帧和置信度门槛。
- K230只输出测量结果，不直接发送飞控动作；F4计算出的相对位置只在飞机内部使用，
  除非后续另行定义并统一实现对外消息。

## 9. 小车视觉 VISION_LINE `0x42`，11字节，40 Hz

```text
0 valid:u8                 1 lost_count:u8
2..3 confidence:u16       4..5 lateral_error_mm:i16
6..7 heading_error_deg:i16
8..9 curvature:i16        10 marker_detected:u8
```

大黑点字段当前仅用于诊断；B/C/D/A任务事件由小车里程计产生。

## 10. 飞控、动作和故障消息

### FC_CMD `0x20`

```text
0 mode:u8                  1 command:u8
2..3 target_height_cm:u16  4..5 target_vx_cm_s:i16
6..7 target_vy_cm_s:i16    8..9 target_vz_cm_s:i16
```

### FC_STATE `0x21`

```text
0 state:u8                 1 armed:u8
2..3 height_cm:u16         4..5 vx_cm_s:i16
6..7 vy_cm_s:i16           8..9 vz_cm_s:i16
10..11 battery_mv:u16      12 fault:u8
```

### FC_POSE `0x22`，8字节

```text
0..1 x_cm:i16
2..3 y_cm:i16
4..5 z_cm:i16
6..7 yaw_deg:i16
```

场地坐标原点为左下角，X向右、Y向上、Z为离地高度。该字段用于地面站态势
显示，不替代飞机内部导航坐标；飞机主控负责从内部NED/ENU坐标转换。

### DROP_STATE `0x50`

```text
0 state:u8   0未动作，1释放，2完成，3失败
1 object_mass_g:u8
```

### LAND_STATE `0x51`

```text
0 state:u8   0未开始，1下降，2已着陆，3停留，4完成，5失败
1..2 stable_time_s:u16
```

### FAULT `0x60`

```text
0 source:u8
1 fault_code:u8
2 severity:u8   1提示，2警告，3错误
```

## 11. 周期、超时和安全

| 消息 | 周期/要求 |
|---|---:|
| VISION_PIXEL | 50～100 ms |
| VISION_LINE | 25 ms |
| TASK_STATE | 100 ms |
| CAR_STATE、CAR_POSE、CAR_DIAGNOSTIC | 100 ms |
| TRACK_EVENT/C_ENTER | 动作区内100 ms |
| HEARTBEAT | 1000 ms |

- 飞机视觉超过300 ms未更新：置无效并执行视觉丢失策略。
- 飞控状态超过500 ms未更新：停止发送新的运动动作，进入飞控安全策略。
- 地面站离线不影响飞机和小车完成正常任务。
- 正式任务仅允许地面站查询和 `TASK_ABORT`，不提供手动驾驶；小车仅允许通过实体K4
  发送安全中止，不允许由此扩展远程手动驾驶。
- UART接收建议DMA+空闲中断；中断只搬运数据，主循环完成解析和业务分发。
