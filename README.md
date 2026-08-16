# 2026 全国大学生电子设计竞赛 D 题｜陆空协同无人机系统

本仓库是 2026 年全国大学生电子设计竞赛 D 题项目的开源代码归档。项目获得省级一等奖，赛场成绩接近满分。

系统由循线小车、无人机任务主控、机载/车载 K230 视觉、透明串口无线链路及图形化地面站组成。代码按设备职责拆分，通信接口集中维护，便于复现、移植和继续开发。

> 本仓库面向竞赛复盘与工程学习。无人机与移动平台具有安全风险，请在断桨、架空或空旷受控环境中验证；不可直接用于载人、公共区域或其他安全关键场景。

## 项目能力

- 小车基于 STM32F407、双编码器与 TB6612 完成速度闭环、循线、里程计及赛道事件触发。
- 车载 K230 以 40 Hz 输出黑线横向误差、航向误差、曲率和标记诊断。
- 飞机 STM32F407 协调飞控、天空端 K230 与小车任务消息，支持跟随、抛投与动态降落流程。
- 地面站通过两路透明串口显示无人机/小车遥测、轨迹、事件和 CSV 日志。
- LXS1 协议提供 C 与 Python 实现及单元测试，统一多节点帧格式和任务消息。

## 仓库结构

```text
Air/                           无人机端
  STM32/AirF407/               飞机任务主控（CubeMX / Keil）
  K230/                        平台识别与视觉跟踪脚本
  Lingxiao/                    凌霄飞控工程及其适配代码
Car/                           小车端
  STM32/CarF407/               小车主控（CubeMX / CMake）
  K230/                        黑线视觉与串口输出
  Hardware/                    PCB、BOM、接线说明和 Gerber
GroundStation/                 地面站
  Software/                    Python 图形化地面站及测试
  Hardware/                    地面站硬件工程
LXS1_通信协议/                  公共通信协议、C/Python 库和测试
docs/architecture/             任务状态机（Markdown 与 YAML）
```

## 核心文档

- [LXS1 通信协议](LXS1_通信协议/LXS1_PROTOCOL.md)：帧格式、节点地址、消息号和载荷定义。
- [任务状态机](docs/architecture/mission-state-machine.md)：设备职责、状态流转和故障策略。
- [机器可读状态机](docs/architecture/mission-state-machine.yaml)：状态机的 YAML 描述。（给AI看的版本）
- [小车 K230 使用说明](Car/K230/README.md)：相机、显示、标定和部署。
- [地面站使用说明](GroundStation/Software/README.md)：安装、串口和界面说明。

## 任务流程与参数

小车负责启动、循线和赛道里程事件；飞机主控负责全局任务状态；地面站仅显示、记录和紧急中止。标准流程如下：

```text
A 启动 → B 事件 → C 减速并执行飞机动作 → D 恢复/返航 → A 停车
```

| 参数 | 标定值 |
|---|---:|
| 正常 / 动作 / 重捕获速度 | 150 / 80 / 40 mm/s |
| B / C / D / A 累计里程 | 1500 / 3856 / 5356 / 7712 mm |
| 里程校准系数 | 1.05187 |
| 安全启动延时 | 10 s |

赛道阶段以小车里程计为主判据，视觉大黑点只用于诊断和终点确认辅助。

## 构建与运行

### 小车 STM32

需要已安装 `arm-none-eabi-gcc` 与 CMake。仓库根目录下执行：

```powershell
Push-Location Car/STM32/CarF407
cmake --preset Debug
cmake --build --preset Debug
Pop-Location
```

也可直接用 STM32CubeMX/Keil 打开 `Car/STM32/CarF407/CarF407.ioc`。小车 K230 部署时，将 `main.py`、`line_vision.py`、`vision_config.py` 和 `lxs1_uart.py` 复制到模块同一目录。

### 地面站

需要 Python 3.10+：

```powershell
Set-Location GroundStation/Software
py -m pip install -r requirements.txt
py main.py
```

Windows 可直接运行 `启动地面站.bat`；Ubuntu 可运行 `sh start_linux.sh`。

## 测试

以下测试无需连接硬件：

```powershell
Push-Location Car/K230; py -B -m unittest discover -s tests -v; Pop-Location
Push-Location LXS1_通信协议; py -B -m unittest discover -s tests -v; Pop-Location
Push-Location GroundStation/Software; py -B -m unittest discover -s tests -v; Pop-Location
```

## 开源说明

本仓库中竞赛团队拥有版权的原创代码与文档采用 [MIT License](LICENSE) 发布。`Air/Lingxiao`、STM32 HAL/CMSIS 与工程内随附的第三方文件不受该授权覆盖，详见 [第三方组件与许可说明](THIRD_PARTY_NOTICES.md)；在再分发或商用前请自行核对上游许可。

比赛期的任务书、人员交付清单、联调汇报和 AI 对接材料已从公开仓库移除。
