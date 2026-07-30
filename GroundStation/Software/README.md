# LXS1 陆空协同地面站

面向 2026 电赛 D 题的离线地面站。程序可通过透明无线串口接收 LXS1
协议遥测，在赛题场地图上叠加小车与无人机位置，并显示任务、飞控、抛投、
降落和故障状态。程序不需要联网。

## 功能

- Windows / Ubuntu 图形界面
- 串口自动扫描、连接、断线提示和收发计数
- LXS1 增量解析，可处理粘包、分包和错误帧
- 打印用场地图实时轨迹、无人机/小车位置与航向
- 任务计时、飞控状态、电池、速度、里程、圈数、链路新鲜度
- 事件日志、原始帧监视、CSV 遥测自动记录
- 有二次确认的广播终止命令
- 无硬件演示模式，可随时重置并从 A 点重新开始

## 安装与启动

需要 Python 3.10 或更高版本。

Windows PowerShell：

```powershell
cd D:\Lab\Program\2026\GroundBase\Software
py -m pip install -r requirements.txt
py main.py
```

Windows 下也可以直接双击 `启动地面站.bat`。它会在首次运行时自动创建
`.venv` 虚拟环境并安装依赖，后续双击即可启动。

Ubuntu：

```bash
sudo apt install python3 python3-tk
cd GroundBase/Software
python3 -m pip install -r requirements.txt
python3 main.py
```

也可以运行 `start_windows.bat` 或 `sh start_linux.sh`。首次启动且未安装
`pyserial` 时，演示模式仍可使用，但串口连接不可用。

## 串口与坐标约定

- 默认串口参数：115200、8 数据位、无校验、1 停止位。
- 地面站地址为 `0x06`，终止命令发往广播地址 `0xFF`。
- `TASK_ABORT`按LXS1 v1.1发送当前 `run_id`和 `reason=1`。
- 地图坐标原点为场地左下角，X 向右（0~400 cm），Y 向上（0~500 cm）。
- 小车显示位置不直接采用 `CAR_POSE`，而是用累计里程沿赛题轨迹换算。
  起点为 A，方向为 A→B→C→D→A，每圈约 771.24 cm。
- `CAR_STATE.path_cm` 和 `TASK_STATE.car_path_mm` 均可驱动里程位置更新。
- 当前小车固件的 13 字节 `CAR_STATE` 使用
  `run_id/state/speed_mode/line_valid/speed_mm_s/fault/path_mm/lap_complete`
  布局；地面站优先采用其中的 `path_mm`，同时兼容旧版 7 字节帧。
- 协议文档没有给出 `FC_POSE` 的数据定义。本程序兼容常用布局
  `<hhh h>`：`x_cm, y_cm, z_cm, yaw_deg`；至少 4 字节时也能显示 x/y。
  飞机主控应采用该布局，或在 `ground_station/telemetry.py` 中调整。

每次启动程序都会在 `logs/` 中创建 CSV 遥测文件。该目录不会上传网络。

## 验证

```bash
python -m unittest discover -s tests -v
```

演示模式可选择“抛投任务”或“动态起降”。小车按 A→B→C→D→A 的顺序
沿赛题闭合轨迹顺时针运行；抛投、降落小车和平台停留均安排在 C→D 段，
动作结束后无人机返航 H 点，小车继续行驶至 A 点才完成。
