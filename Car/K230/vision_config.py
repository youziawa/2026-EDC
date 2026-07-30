"""小车 K230 视觉参数。

现场调参优先修改本文件，算法和通信代码通常不需要改动。
"""

# ------------------------------ 图像配置 ------------------------------

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# 请求传感器选择较高帧率的输入模式，再由 ISP 输出 QVGA。
# 不支持这些构造参数的旧固件会自动退回 Sensor() 默认模式。
CAMERA_MODE_WIDTH = 640
CAMERA_MODE_HEIGHT = 480
CAMERA_MODE_FPS = 60

# 摄像头安装方向不同时修改这两个开关。
CAMERA_HMIRROR = False
CAMERA_VFLIP = False

# 0 表示使用自动曝光；大于 0 时尝试锁定为对应的微秒数。
# 固件和传感器支持时，现场确定曝光值后锁定通常能进一步降低阈值抖动。
MANUAL_EXPOSURE_US = 0

# 相机光轴对应的图像横坐标。摄像头没有正对车体中心时需要标定。
OPTICAL_CENTER_X = IMAGE_WIDTH // 2

# 从近到远的 5 条水平采样带中心位置。
STRIP_CENTER_Y = (218, 190, 162, 134, 106)
STRIP_HEIGHT = 22

# 跟踪成功时只在上一次位置附近搜索，以提高速度并抑制背景干扰。
TRACK_SEARCH_HALF_WIDTH = 92
LOST_EXPAND_PIXELS = 24

# ------------------------------ 黑线分割 ------------------------------

# 默认启用 Otsu 自适应阈值。赛场光照稳定时也可关闭并手动设置 FIXED_THRESHOLD。
ADAPTIVE_THRESHOLD = True
FIXED_THRESHOLD = 82
THRESHOLD_MIN = 28
THRESHOLD_MAX = 155
THRESHOLD_OFFSET = -4
THRESHOLD_UPDATE_FRAMES = 5
THRESHOLD_FILTER_ALPHA = 0.30

# find_blobs 参数。黑线太细时可适当降低，噪点多时适当提高。
X_STRIDE = 2
Y_STRIDE = 2
MIN_BLOB_PIXELS = 28
MIN_BLOB_AREA = 36
MIN_LINE_WIDTH = 3
MAX_LINE_WIDTH = 120
MAX_LINE_HEIGHT = STRIP_HEIGHT
MERGE_MARGIN = 2

# 候选区域距离上一次位置越远，得分越低。
POSITION_PENALTY = 2
WIDTH_PENALTY = 1

# ------------------------------ 有效性与滤波 ------------------------------

MIN_VALID_STRIPS = 3
MIN_CONFIDENCE = 380
VALID_CONFIRM_FRAMES = 2

# 输出一阶低通。数值越大响应越快，越小越平稳。
LATERAL_FILTER_ALPHA = 0.42
HEADING_FILTER_ALPHA = 0.38
CURVATURE_FILTER_ALPHA = 0.32

# 每帧允许的最大变化，防止偶发误识别导致控制量突跳。
MAX_LATERAL_STEP_MM = 75.0
MAX_HEADING_STEP_DEG = 24.0
MAX_CURVATURE_STEP = 4500.0

# ------------------------------ 几何标定 ------------------------------

# 输出横向误差的位置，通常位于车辆前方对应的图像近端。
CONTROL_Y = 214

# 必须根据实际安装高度和俯角标定。
# 横向误差 = 像素偏移 * LATERAL_MM_PER_PIXEL。
LATERAL_MM_PER_PIXEL = 1.00

# 图像纵向每像素近似对应的前向距离，仅用于航向与曲率量纲换算。
FORWARD_MM_PER_PIXEL = 2.00

# ------------------------------ 串口与协议 ------------------------------

UART_CHANNEL = 3
UART_TX_PIN = 32
UART_RX_PIN = 33
UART_BAUDRATE = 115200

LXS1_SRC = 0x05       # 小车 K230
LXS1_DST = 0x04       # 小车 STM32F407
SEND_PERIOD_MS = 25   # 40 Hz

# ------------------------------ 运行诊断 ------------------------------

# 亚博 K230 配套 2.4 英寸 ST7701 屏为 640x480。
DISPLAY_ENABLED = True
DISPLAY_TYPE = "ST7701"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
DISPLAY_FPS = 30
DISPLAY_TO_IDE = False
DISPLAY_PERIOD_MS = 100  # 约 15 Hz，避免调试显示拖慢识别和串口

# 正式比赛可关闭屏幕；终端日志也建议保持 False。
DEBUG_PRINT = False
DEBUG_DRAW = False
DEBUG_PRINT_PERIOD_MS = 1000

# 周期性回收由图像算法产生的临时对象，避免长时间运行后抖动。
GC_PERIOD_FRAMES = 160
