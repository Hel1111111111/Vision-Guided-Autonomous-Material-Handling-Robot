import sensor
import image
import time
import math
from machine import UART


# =========================
# 串口初始化
# 和你原来成功的主车摄像头代码保持一致
# =========================
uart = UART(2, 115200, timeout_char=2)


# =========================
# 摄像头初始化
# =========================

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)

sensor.skip_frames(time=1000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 光斑过大就调小，识别不到就调大
sensor.set_auto_exposure(False, exposure_us=500)

sensor.skip_frames(time=500)

clock = time.clock()


# =========================
# 参数区
# =========================

IMG_W = 320
IMG_H = 240
IMG_CX = IMG_W // 2
IMG_CY = IMG_H // 2

# 红外亮点阈值
IR_THRESHOLD = (200, 255)

# 搜索区域
# 你现在灯基本在画面下半部分，所以 ROI 从 y=120 开始
ROI = (0, 120, 320, 120)

MIN_PIXELS = 3
MIN_AREA = 3

MAX_W = 70
MAX_H = 70

# 两灯左右排列
MIN_X_GAP = 5
MAX_Y_DIFF = 50

# 期望两灯间距，需要你后续标定
DESIRED_GAP = 180
DEBUG = True


# =========================
# 滤波参数
# =========================

FILTER_ALPHA = 0.25

target_x_filter = None
target_y_filter = None
gap_filter = None


# =========================
# 工具函数
# =========================

def get_rect_center(b):
    """
    使用外接矩形中心，而不是 b.cx()/b.cy()。
    红外灯光斑大、内部亮度不均时，矩形中心通常更稳定。
    """
    cx = b.x() + b.w() // 2
    cy = b.y() + b.h() // 2
    return cx, cy


def is_valid_blob(b):
    if b.pixels() < MIN_PIXELS:
        return False

    if b.area() < MIN_AREA:
        return False

    if b.w() < 2 or b.h() < 2:
        return False

    if b.w() > MAX_W or b.h() > MAX_H:
        return False

    return True


def choose_two_lamps(blobs):
    candidates = []

    for b in blobs:
        if is_valid_blob(b):
            candidates.append(b)

    if len(candidates) < 2:
        return None, candidates

    best_pair = None
    best_score = -999999

    for i in range(len(candidates)):
        for j in range(i + 1, len(candidates)):
            b1 = candidates[i]
            b2 = candidates[j]

            x1, y1 = get_rect_center(b1)
            x2, y2 = get_rect_center(b2)

            dx = abs(x2 - x1)
            dy = abs(y2 - y1)

            # 左右灯必须有一定水平间距
            if dx < MIN_X_GAP:
                continue

            # 左右灯高度差不能过大
            if dy > MAX_Y_DIFF:
                continue

            p1 = b1.pixels()
            p2 = b2.pixels()

            # 防止选到某个灯内部的小碎块
            big = max(p1, p2)
            small = min(p1, p2)

            if small < big * 0.35:
                continue

            size_diff = abs(p1 - p2)

            # 主灯一般在画面下方，所以 y 越大越可信
            y_score = y1 + y2

            # 面积越大越好，越靠下越好，高度差越小越好，大小越接近越好
            score = p1 + p2 + y_score * 2 - dy * 5 - size_diff * 0.2

            if score > best_score:
                best_score = score
                best_pair = (b1, b2)

    if best_pair is None:
        return None, candidates

    a, b = best_pair

    ax, ay = get_rect_center(a)
    bx, by = get_rect_center(b)

    if ax < bx:
        left_lamp = a
        right_lamp = b
    else:
        left_lamp = b
        right_lamp = a

    return (left_lamp, right_lamp), candidates


# =========================
# 可选：接收主板指令
# =========================
# 为了和你原来成功的摄像头代码结构一致，这里保留一个接收函数。
# 目前红外跟随暂时不需要处理 C,id 指令。
# 如果主板发来 C,xxx，这里只打印，不影响识别。

def parse_command():
    if uart.any():
        try:
            line = uart.readline()
            if line:
                cmd = line.decode().strip()
                if cmd:
                    pass  # print("CMD:", cmd)
        except Exception as e:
            pass  # print("UART CMD ERR:", e)


# =========================
# 主循环
# =========================

frame_count = 0

while True:
    clock.tick()
    frame_count += 1

    # 读取主板可能发来的指令
    parse_command()

    img = sensor.snapshot()

    #img.draw_rectangle(ROI, color=80)
    #img.draw_line(IMG_CX, 0, IMG_CX, IMG_H, color=80)

    blobs = img.find_blobs(
        [IR_THRESHOLD],
        roi=ROI,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=False,
        margin=0
    )

    pair, candidates = choose_two_lamps(blobs)

    # 画所有候选点
    '''for b in candidates:
        cx, cy = get_rect_center(b)
        img.draw_rectangle(b.rect(), color=150)
        img.draw_cross(cx, cy, color=150)'''

    if pair is not None:
        left_lamp, right_lamp = pair

        x1, y1 = get_rect_center(left_lamp)
        x2, y2 = get_rect_center(right_lamp)

        target_x_raw = (x1 + x2) // 2
        target_y_raw = (y1 + y2) // 2

        dx = x2 - x1
        dy = y2 - y1

        gap_raw = target_y_raw

        # 一阶滤波，减少跳动
        if target_x_filter is None:
            target_x_filter = target_x_raw
            target_y_filter = target_y_raw
            gap_filter = gap_raw
        else:
            target_x_filter = target_x_filter * (1 - FILTER_ALPHA) + target_x_raw * FILTER_ALPHA
            target_y_filter = target_y_filter * (1 - FILTER_ALPHA) + target_y_raw * FILTER_ALPHA
            gap_filter = gap_filter * (1 - FILTER_ALPHA) + gap_raw * FILTER_ALPHA

        target_x = int(target_x_filter)
        target_y = int(target_y_filter)
        gap = gap_filter

        err_x = target_x - IMG_CX
        err_gap = DESIRED_GAP - gap

        angle = math.atan2(dy, dx) * 57.2958

        #img.draw_rectangle(left_lamp.rect(), color=255)
        #img.draw_rectangle(right_lamp.rect(), color=255)

        #img.draw_cross(x1, y1, color=255)
        #img.draw_cross(x2, y2, color=255)
        #img.draw_line(x1, y1, x2, y2, color=255)

        #img.draw_cross(target_x, target_y, color=255)

        #img.draw_string(0, 0, "OK", color=255)
        #img.draw_string(0, 12, "EX:%d" % int(err_x), color=255)
        #img.draw_string(0, 24, "GAP:%d" % int(gap), color=255)
        #img.draw_string(0, 36, "ANG:%d" % int(angle), color=255)

        msg = "$IR,2,%d,%d,%d,%d,%d,%d\n" % (
            target_x,
            target_y,
            int(err_x),
            int(gap),
            int(err_gap),
            int(angle)
        )

        # USB 调试终端打印
        #if DEBUG and frame_count % 3 == 0:
        #    print(msg)

        # 物理 UART2 发送给主板
        uart.write(msg)

    else:
        if len(candidates) == 1:
            b = candidates[0]

            bx, by = get_rect_center(b)
            err_x = bx - IMG_CX

            #img.draw_rectangle(b.rect(), color=255)
            #img.draw_cross(bx, by, color=255)

            #img.draw_string(0, 0, "ONE", color=255)

            msg = "$IR,1,%d,%d,%d,0,0,0\n" % (
                bx,
                by,
                int(err_x)
            )

            if DEBUG and frame_count % 3 == 0:
                print(msg)

            uart.write(msg)

        else:
            img.draw_string(0, 0, "NO IR", color=255)

            msg = "$IR,0,0,0,0,0,0,0\n"

            if DEBUG and frame_count % 3 == 0:
                print(msg)

            uart.write(msg)
