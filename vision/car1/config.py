# -*- coding: utf-8 -*-
"""
五类物品识别 + 原版色块二次纠正。

本工程：
    保留五类模型识别
    恢复原代码的整行色块二次纠正
    保留黄色边线 STOP
    保留 UART2 主板通信

本工程不包含：
    绿色区域和类别 5
    红砖识别
"""

# =========================================================
# 工作模式
# =========================================================
MODE_IDLE = 0
MODE_OBJECT = 1

MODE_NAME = {
    MODE_IDLE: "IDLE",
    MODE_OBJECT: "OBJECT",
}

# 直接运行 main.py 时立即识别。
# 正式比赛想等待主板命令，可改为 MODE_IDLE。
START_MODE = MODE_OBJECT

# -1：五类全部允许；0~4：只识别指定类别。
START_DESIRED_CLASS = -1

# =========================================================
# UART
# =========================================================
UART_ID = 2
UART_BAUD = 115200
UART_TIMEOUT_CHAR = 2

# =========================================================
# 模型
# =========================================================
MODEL_PATH = "/sd/yolo3_iou_smartcar_final_with_post_processing.tflite"

LABELS = [
    "sandbag.red",     # 0
    "sandbag.blue",    # 1
    "tennis",          # 2
    "teddy.white",     # 3
    "teddy.brown",     # 4
]

SCORE_THRESH = 0.40

MIN_TARGET_H = 0.05
MIN_TARGET_BOTTOM_Y = 0.20

TRACK_MATCH_CX = 0.12
TRACK_MATCH_BOTTOM_Y = 0.12

TARGET_BOTTOM_WEIGHT = 100.0
TARGET_CENTER_WEIGHT = 0.2
TARGET_KEEP_BONUS = 0.5

# =========================================================
# 原版色块二次纠正
# =========================================================
ENABLE_ORIGINAL_COLOR_CORRECTION = True

# 对应原代码：
#     px1 = int((x1 - 0.1) * img.width())
ORIGINAL_X_OFFSET = 0.10

COLOR_PIXELS_THRESH = 20
COLOR_AREA_THRESH = 20

# 按你原程序中的阈值原样恢复。
THRESHOLDS = {
    0: [
        (19, 63, 31, 111, 115, 8),
        (24, 59, -34, -4, -52, -6),
    ],
    1: [
        (46, 92, -52, 13, -82, -38),
        (40, 94, -55, 12, -82, -38),
        (26, 71, -42, 11, -86, -33)
    ],
    2: [
        #(81, 100, -97, 2, 0, 109),
        #(84, 100, -79, -4, 37, 127),
        (39, 100, -78, -15, 10, 127),
        (62, 100, -47, 9, 23, 112),
        (60, 98, -73, -22, 21, 118)
    ],
    3: [
        #(88, 100, -16, 8, -12, 54),
        #(81, 100, -7, 6, -3, 127),
        #(0, 100, -79, -4, 37, 127),
        #(86, 100, -2, 7, -5, 9),
        (45, 97, -20, 42, 4, 63)
    ],
    4: [
        (28, 70, 3, 127, 11, 127),
        (5, 69, -22, 44, 11, 127)
    ],
}

# =========================================================
# 下部 ROI 模式
# =========================================================
# 主板发送 R,1 后，只接受 bottom_y 不小于该值的目标。
# 主板发送 R,0 后恢复全画面。
# 先从 0.56 开始，现场根据前排/后排物体的 bottom_y 分布调整。
LOWER_ROI_MIN_BOTTOM_Y = 0.45

# =========================================================
# 鲁棒时域滤波
# =========================================================
# 新目标第一次出现时需要连续确认。
FILTER_CONFIRM_FRAMES = 2

# 已锁定目标突然跳到另一目标时，需要更长确认。
FILTER_SWITCH_CONFIRM_FRAMES = 5

# 短暂 N 或错误候选时，继续发送上一有效目标的帧数。
FILTER_LOSE_FRAMES = 5

# 判断是否仍是同一目标。
FILTER_MATCH_CX = 0.16
FILTER_MATCH_BOTTOM_Y = 0.16
FILTER_MIN_SIZE_RATIO = 0.55

# 下部 ROI/接近阶段：目标应总体变近。
# bottom_y 突然减小太多或尺寸突然变小，通常是后排同类物体。
NEAR_MAX_BACKWARD_BOTTOM_Y = 0.05
NEAR_MIN_HEIGHT_RATIO = 0.70

# 坐标平滑系数。
FILTER_SMOOTH_ALPHA = 0.55

# =========================================================
# 黄色边线
# =========================================================
ENABLE_YELLOW_LINE = True

# 是否显示两个青色 ROI 框。
DRAW_YELLOW_ROI = False

YELLOW_THRESHOLDS = [
    #(70, 98, -23, 31, 48, 127),
    #(49, 100, -22, 67, 31, 127),
    (49, 83, -48, -6, 31, 127),
    (49, 83, -49, 9, 56, 127),
    (66, 82, -24, 7, 21, 116),
    (96, 100, -82, -17, 78, 93),
    (68, 93, -20, 34, 63, 119),
]

YELLOW_ROIS = [
    (0, 160, 110, 80),
    (200, 160, 110, 80),
]

YELLOW_PIXELS_THRESH = 200
YELLOW_AREA_THRESH = 200
YELLOW_STOP_LINE_Y = 160
YELLOW_MIN_W = 95
YELLOW_MIN_H = 10
YELLOW_CONFIRM_FRAMES = 1



# =========================================================
# 黄线航向标定模式
# =========================================================
# 主板发送：
#     IMU,1：进入黄线航向测量模式
#     IMU,0：退出，恢复普通物体/STOP 识别
#
# 航向模式下摄像头只检测黄线方向，并发送：
#     Y,angle_x100,quality,cy_x1000
# 没有得到稳定黄线时发送：
#     YN
#
# angle 为黄线相对画面水平线的角度，单位为度。
# 最终发送时乘以 100。
YAW_LINE_ROI = (0, 100, 320, 140)

# 用于确认“这确实是一条场地边界线”的色块条件。
YAW_LINE_BLOB_PIXELS_THRESH = 100
YAW_LINE_BLOB_AREA_THRESH = 140
YAW_LINE_MIN_BLOB_W = 45
YAW_LINE_MIN_BLOB_H = 4
YAW_LINE_MIN_BLOB_ASPECT = 2.0

# 一个完整边界可能被被推动物体遮挡，左右形成多个色块；
# 将所有满足条件的长条色块宽度相加进行判断。
YAW_LINE_MIN_SUPPORT_WIDTH = 100
YAW_LINE_MIN_SUPPORT_PIXELS = 220

# 对所有黄色像素做直线回归时的最低条件。
YAW_LINE_REG_PIXELS_THRESH = 120
YAW_LINE_REG_AREA_THRESH = 180
YAW_LINE_MIN_LENGTH = 80
YAW_LINE_MIN_MAGNITUDE = 5

# 正常用于校正航向时，黄线不应在画面里倾斜得过于夸张。
YAW_LINE_MAX_ABS_ANGLE_DEG = 35.0

# 摄像头端连续帧防抖。
YAW_LINE_HISTORY_SIZE = 5
YAW_LINE_CONFIRM_FRAMES = 3
YAW_LINE_STABLE_SPREAD_DEG = 3.0

# 调试绘制：
# 白线：回归得到的黄线方向；
# 绿框：参与回归的长条黄色色块。
DRAW_YAW_LINE_DEBUG = True

# =========================================================
# 调试
# =========================================================
DRAW_DEBUG = True
PRINT_DEBUG = True
IDLE_PRINT_INTERVAL_FRAMES = 30
