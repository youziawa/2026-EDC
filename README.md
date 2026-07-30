# 2026电赛D题：陆空协同无人机系统

本仓库保存当前可联调基线，包括小车STM32/K230代码、通信协议、统一状态机、
硬件资料、可烧录固件和飞行器对接说明。

## 对接人员先看

- [飞行器联调通信对接汇报](飞行器联调通信对接汇报.md)
- [地面站对接说明](GroundStation/对接说明.md)
- [给队友AI的任务书](GroundStation/给队友AI的任务书.md)
- [LXS1通信协议](LXS1_通信协议/LXS1_PROTOCOL.md)
- [统一状态机（队员版）](陆空协同统一状态机.md)
- [统一状态机（AI/YAML版）](land_air_state_machine.yaml)

## 目录

```text
Car/
  STM32/CarF407/       小车主控完整CMake/CubeMX工程
    firmware/          已验证的可烧录ELF
  K230/                小车巡线视觉
  Hardware/            接线、PCB、BOM和Gerber
GroundStation/
  Hardware/            地面站硬件工程
  Software/            Windows/Ubuntu图形化地面站
LXS1_通信协议/          C/Python协议库、测试和诊断工具
skills/                AI通信协议辅助资料
```

## 当前关键参数

```text
正常速度：150 mm/s
动作速度：80 mm/s
重捕获速度：40 mm/s
B/C/D/A累计里程：1500 / 3856 / 5356 / 7712 mm
里程校准系数：1.05187
安全启动延时：10 s
```

任务事件以小车里程计为主判据；K230大黑点识别仅作诊断。正确流程为：

```text
A启动 → B事件 → C减速/飞机动作 → D恢复/超时返航 → A停车
```

## 构建与烧录

```powershell
cd Car\STM32\CarF407
cmake --preset Debug
cmake --build --preset Debug
```

已构建固件：[CarF407.elf](Car/STM32/CarF407/firmware/CarF407.elf)

K230需复制：

```text
Car/K230/main.py
Car/K230/line_vision.py
Car/K230/vision_config.py
Car/K230/lxs1_uart.py
```

## 测试

```powershell
python -B -m unittest discover -s Car\K230\tests -v
python -B -m unittest discover -s LXS1_通信协议\tests -v
python -B -m unittest discover -s GroundStation\Software\tests -t GroundStation\Software -v
```

## 当前限制

- 图形化地面站已实现，支持LXS1 v1.1、小车里程地图定位、任务状态、日志、
  `TASK_ABORT`和无硬件演示。入口为 `GroundStation/Software/main.py`。
- 地面站GUI当前接收单个串口字节流；正式双链路需要地面站F103把飞机和小车
  两路ECB02原始LXS1帧汇聚到一个PC串口，或后续扩展GUI同时打开两个COM口。
- 飞机主控、凌霄飞控适配层和天空端K230代码由对应负责人对接实现。
- ECB02为点对点链路，飞机、小车、地面站各需要两块模块。
