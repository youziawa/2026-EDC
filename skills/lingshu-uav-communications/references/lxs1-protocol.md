# LXS1 AI 工作参考

权威定义是仓库根目录：

- `LXS1_通信协议/LXS1_PROTOCOL.md`
- `land_air_state_machine.yaml`
- `飞行器联调通信对接汇报.md`

修改消息字段前必须读取以上文件，不得从旧KGS1/RTSP代码推断。

## 固定帧

```text
AA 55 | LEN | SRC | DST | MSG | DATA | 0D 0A
LEN = 3 + DATA长度
```

- 小端序，DATA最大64字节，无CRC、ACK、序号和重试字段。
- 节点：`01`飞控适配、`02`飞机主控、`03`天空K230、`04`小车主控、
  `05`小车K230、`06`地面站、`FF`广播。

## 当前关键消息

```text
10 TASK_START      小车→飞机/地面站，6字节
11 TASK_ABORT      飞机/地面站→广播
12 TASK_STATE      飞机→小车/地面站，14字节，100 ms
31 CAR_STATE       小车→飞机/地面站，13字节，100 ms
32 CAR_POSE        小车→飞机/地面站，11字节，100 ms
33 TRACK_EVENT     小车→飞机/地面站，6字节
34 CAR_DIAGNOSTIC  小车→地面站，52字节
40 VISION_PIXEL    天空K230→飞机F4本地链路，12字节，不转发
41 VISION_LANDMARK 保留，当前禁止发送
42 VISION_LINE     小车K230→小车，11字节
50 DROP_STATE      飞机→地面站
51 LAND_STATE      飞机→地面站
60 FAULT           任意→对应主控/地面站
```

不要改变：

```text
TASK_START = run_id:u8, task_mode:u8, normal_speed:u16, action_speed:u16
TRACK_EVENT = run_id:u8, event:u8, path_mm:u32
VISION_PIXEL = valid:u8, kind:u8, confidence:u16, dx_px/dy_px/radius_px/vision_mode:i16
```

全局状态为0～13，`RETURN=9`；事件为 `B=1,C=2,D=3,A=4`。

F4以飞控实际高度将像素测量换算为机体FRD厘米误差；像素帧不得离开机载K230—F4链路。

## 当前安全规则

- 天空视觉超过300 ms无更新时无效。
- 小车K230链路超过180 ms无更新时停止；内容无效时最长3秒低速重捕获。
- 飞控状态超过500 ms无更新时进入安全策略。
- 地面站离线2秒只影响显示，不中断任务。
- 所有副作用按 `run_id`幂等，`TASK_STATE`和C区间 `C_ENTER`必须周期发送。
- ECB02仅点对点：飞机、小车、地面站各两块，共三条无线链路。
- 视觉节点只输出测量，不能直接控制飞控、电机、投放器或状态机。
