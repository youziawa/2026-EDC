# 给队友AI的陆空协同对接任务书

把本文件完整交给负责飞机、天空视觉或地面站汇聚固件的AI。执行前先阅读：

1. `LXS1_通信协议/LXS1_PROTOCOL.md`
2. `land_air_state_machine.yaml`
3. `飞行器联调通信对接汇报.md`
4. `GroundStation/对接说明.md`

## 总目标

完成飞机主控、天空端K230和地面站双链路汇聚，使现有小车与
`GroundStation/Software`可以按LXS1 v1.1联调。不得自行更改消息号、字段
顺序、字节序、节点地址或帧格式。

## 不允许改动的协议基线

```text
帧：AA 55 | LEN | SRC | DST | MSG | DATA | 0D 0A
串口：115200 8N1
多字节：小端序
飞机主控：0x02
天空K230：0x03
小车主控：0x04
地面站：0x06
广播：0xFF
```

当前协议无CRC、无ACK，不得单端增加校验或转义。所有有副作用的任务动作按
`run_id`幂等，同一 `run_id` 的重复帧不得重复起飞、投放或降落。

## 任务A：飞机主控/状态机

实现或检查以下逻辑：

1. 接收小车 `TASK_START 0x10`、`TRACK_EVENT 0x33`、
   `CAR_STATE 0x31`和 `CAR_POSE 0x32`。
2. 新 `run_id` 才初始化任务。`C_ENTER`触发DROP或LAND_CAR；
   到D仍未完成必须取消动作并RETURN。
3. 每100 ms同时向小车和地面站发送14字节 `TASK_STATE`：

```text
run_id:u8
task_mode:u8
global_state:u8
result:u8
fault_code:u16
elapsed_ms:u32
car_path_mm:u32
```

4. 全局状态码严格使用：

```text
0 IDLE, 1 TAKEOFF, 2 HOVER_3S, 3 SEARCH, 4 FOLLOW,
5 DROP, 6 LAND_CAR, 7 WAIT_5S, 8 RETAKEOFF,
9 RETURN, 10 LAND_H, 11 DONE, 12 ABORT, 13 FAULT
```

5. 收到地面站 `TASK_ABORT 0x11` 时校验当前 `run_id`，飞机进入安全降落，
   并向小车转发或广播终止。
6. 周期发送13字节 `FC_STATE 0x21`和8字节 `FC_POSE 0x22`：

```text
FC_POSE = x_cm:i16, y_cm:i16, z_cm:i16, yaw_deg:i16
```

7. 抛投执行器发送 `DROP_STATE 0x50`；动态降落发送
   `LAND_STATE 0x51`；故障发送 `FAULT 0x60`。
8. 地面站离线不能改变飞机任务状态。

完成标准：用Python协议解析器回放一轮任务，地面站显示的
`TAKEOFF→...→DONE`与飞机内部状态逐帧一致。

## 任务B：天空端K230

1. 节点地址固定 `SRC=0x03`，只向飞机主控 `DST=0x02`发送测量。
2. 10～20 Hz连续发送 `VISION_TARGET 0x40`和所需的
   `VISION_LANDMARK 0x41`。
3. 坐标采用机体系FRD：X前、Y右、Z下，距离cm，沿Z向下看顺时针yaw为正。
4. 无目标时仍周期发送，`valid=0`且所有误差字段清零。
5. 视觉只输出测量，不直接控制飞控、投放器或状态机。

完成标准：人工向前、向右移动目标时dx/dy符号正确；遮挡超过300 ms后飞机
按视觉无效处理，不继续使用旧误差。

## 任务C：地面站F103双链路汇聚

GUI当前只打开一个PC串口。F103需把飞机和小车两路ECB02汇聚：

1. UART_A连接飞机ECB02，UART_B连接小车ECB02，UART_PC连接电脑。
2. 两路输入各自独立按LXS1帧头、LEN和帧尾收完整帧。
3. 以“完整帧”为单位排队转发到PC，禁止两路字节交叉。
4. 不修改帧内容，不重新设置SRC/DST，不增加CRC。
5. PC发来的广播 `TASK_ABORT` 完整转发到UART_A和UART_B。
6. 单路异常或断开不得阻塞另一路。

如果不实现F103汇聚，替代任务是修改
`GroundStation/Software/ground_station/transport.py`和`app.py`，支持同时
选择、打开两个COM口，并把两路字节送入同一个线程安全解析队列。

完成标准：飞机与小车同时10 Hz发帧持续10分钟，地面站解析错误计数不增长，
两路节点都能持续刷新，终止帧能同时到达飞机和小车。

## 任务D：联调测试

按以下顺序执行，不得一开始安装螺旋桨：

1. 纯串口回环验证所有消息长度、字节序、SRC/DST。
2. 小车抬轮运行，验证 `path_mm` 单调增加且地面站从A向B移动。
3. 人工注入C/D事件，验证动作仅发生在C—D段。
4. 无桨验证完整抛投和动态起降状态流。
5. 验证重复 `TASK_START/C_ENTER` 不产生重复副作用。
6. 验证视觉断流300 ms、飞控断流500 ms和地面站断开。
7. 最后才进行系留和低高度测试。

## 修改代码后的交付要求

- 说明修改了哪些文件以及为什么。
- 给出精确的串口/UART编号和TX/RX引脚。
- 提供至少一组真实LXS1十六进制帧和解析结果。
- 运行相关单元测试并记录结果。
- 不要只给伪代码；提交能编译/运行的实现。
- 发现协议冲突时停止并报告，不要自行创造第三种格式。
