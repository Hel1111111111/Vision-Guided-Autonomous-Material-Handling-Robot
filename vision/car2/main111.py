import sensor
import image
import time
import tf
import gc
import pyb
from machine import UART

# =========================
# 串口初始化
# =========================
uart = UART(2, 115200, timeout_char=2)

# =========================
# 摄像头初始化
# =========================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)   # 320x240
sensor.skip_frames(time=2000)
clock = time.clock()

# =========================
# 模型路径
# =========================
model_path = '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite'
net = tf.load(model_path)

# =========================
# 类别表
# 0: sandbag.red, 1: sandbag.blue, 2: tennis, 3: teddy.white, 4: teddy.brown
# =========================
labels = ["sandbag.red", "sandbag.blue", "tennis", "teddy.white", "teddy.brown"]

# =========================
# 目标检测阈值
# =========================
SCORE_THRESH = 0.40

# =========================
# 目标类别选择
# -1 表示不限类别
# =========================
desired_class = -1

# =========================
# 黄线及绿色阈值 (修改为支持多组阈值的列表格式)
# =========================
YELLOW_THRESHOLDS = [
    (35, 85, -22, 27, 44, 90),
    (56, 73, 22, 69, 55, 102),
]

GREEN_THRESHOLDS = [
    (59, 67, -92, -28, -128, 127),
    (47, 67, -92, -28, -128, 127),
    (58, 77, -72, -26, 45, 115),
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

yellow_count = 0
YELLOW_COUNT_OK = 1

# =========================
# 目标稳定跟踪参数
# =========================
last_target = None  #当前锁定目标 ID
TARGET_HOLD_MAX = 8   #计数的最大上限。到了8就不会再往上加了，代表“我已经对你非常非常眼熟了”
target_hold_count = 0  #连续跟踪计数器。当代码确认这一帧的物体和上一帧是同一个时，这个计数器就会 +1
TRACK_MATCH_CX = 0.12  #用来判断物体位置变化的
TRACK_MATCH_BY = 0.12

MIN_TARGET_H = 0.05
MIN_TARGET_BOTTOM_Y = 0.20

# =========================
# 【新增】防抖滤波参数 (去噪)
# =========================
FILTER_CONFIRM_FRAMES = 2  # 连续识别到 3 帧同一类别，才确认为新目标（滤除一闪而过的干扰）
FILTER_LOSE_FRAMES = 4     # 连续 2 帧没看到原目标，才判定目标彻底消失（弥补偶尔掉帧）

# 滤波内部状态变量
tracking_label = -1
candidate_label = -1  #考察期目标考察次数
candidate_count = 0   #考察次数
missing_count = 0     #当正在追踪的目标突然不见了，这个变量就开始累加
last_valid_target = None   #保存着目标在“消失前最后一刻”的完美坐标（x, y, w, h）

# =========================================================
# LAB 二次筛选阈值
# =========================================================
THRESHOLDS = {
    0: [ # sandbag.red
        (19, 63, 31, 111, 115, 8),
        (24, 59, -34, -4, -52, -6),
    ],
    1: [ # sandbag.blue
        (33, 54, -37, -5, -43, -9),
        (30, 83, -60, -1, -30, 0),
    ],
    2: [ # tennis
        (46, 100, -25, -5, 56, 116),
        (18, 100, -50, -3, 51, 100),
        (29, 100, -62, 0, 33, 127),
    ],
    3: [ # teddy.white
        (72, 100, -47, 70, 43, 117),
        (89, 100, -36, 37, -4, 92),
        (51, 100, -98, 21, 21, 91),
    ],
    4: [ # teddy.brown
        (0, 98, 14, 48, 35, 127),
        (0, 87, 14, 50, 27, 127),
        (10, 66, 10, 68, 10, 97),
        (0, 67, 21, 48, 27, 121),
    ]
}

COLOR_PIXELS_THRESH = 20
COLOR_AREA_THRESH = 20


def clamp01(v):
    if v < 0: return 0.0
    if v > 1: return 1.0
    return v

def clamp_int(v, low, high):
    if v < low: return low
    if v > high: return high
    return v

def safe_rect_from_norm(x1, y1, x2, y2, img_w, img_h):
    x1 = clamp01(x1)
    y1 = clamp01(y1)
    x2 = clamp01(x2)
    y2 = clamp01(y2)
    if x2 <= x1 or y2 <= y1: return None
    px1 = clamp_int(int(x1 * img_w), 0, img_w - 1)
    py1 = clamp_int(int(y1 * img_h), 0, img_h - 1)
    px2 = clamp_int(int(x2 * img_w), 0, img_w - 1)
    py2 = clamp_int(int(y2 * img_h), 0, img_h - 1)
    if px2 <= px1 or py2 <= py1: return None
    return (px1, py1, px2 - px1, py2 - py1)

def pixel_rect_to_norm(x, y, w, h, img_w, img_h):
    if w <= 0 or h <= 0: return None
    x1 = clamp01(float(x) / img_w)
    y1 = clamp01(float(y) / img_h)
    x2 = clamp01(float(x + w) / img_w)
    y2 = clamp01(float(y + h) / img_h)
    if x2 <= x1 or y2 <= y1: return None
    return (x1, y1, x2, y2)

def parse_command():
    global desired_class
    global tracking_label, candidate_label, candidate_count, missing_count # 引入滤波全局变量

    if uart.any():
        try:
            line = uart.readline()
            if line:
                cmd_str = line.decode().strip()
                changed = False

                if cmd_str.startswith("C,"):
                    desired_class = int(cmd_str.split(",")[1])
                    changed = True
                elif cmd_str in ["0", "1", "2", "3", "4", "5"]:
                    desired_class = int(cmd_str)
                    changed = True
                elif cmd_str.upper() in ["DONE", "ALL IS DONE"]:
                    desired_class = 5
                    changed = True
                elif cmd_str == "-1":
                    desired_class = -1
                    changed = True

                if changed:
                    # 【逻辑新增】收到主板切换目标指令时，立刻清空当前的滤波缓存
                    # 防止上一次的目标残留导致新目标响应迟钝
                    tracking_label = -1
                    candidate_label = -1
                    candidate_count = 0
                    missing_count = 0
                    print(">>> Target Changed To:", desired_class)
        except Exception as e:
            print("UART CMD ERR:", e)


def build_target_dict(x1, y1, x2, y2, label, score):
    return {
        "label": int(label), "score": score,
        "x1": x1, "y1": y1, "x2": x2, "y2": y2,
        "cx": (x1 + x2) / 2.0, "cy": (y1 + y2) / 2.0,
        "w": x2 - x1, "h": y2 - y1, "bottom_y": y2
    }


def choose_best_target(ctrl_detections):
    global last_target, target_hold_count, desired_class

    candidates = []
    for det in ctrl_detections:
        if len(det) == 7:
            x1, y1, x2, y2, label, score, y_diff = det
        else:
            x1, y1, x2, y2, label, score = det
            y_diff = 0

        if score < SCORE_THRESH: continue
        label = int(label)

        if desired_class != -1 and label != desired_class: continue

        x1, y1, x2, y2 = clamp01(x1), clamp01(y1), clamp01(x2), clamp01(y2)
        if x2 <= x1 or y2 <= y1: continue

        t = build_target_dict(x1, y1, x2, y2, label, score)
        t["y_diff"] = y_diff

        if t["h"] < MIN_TARGET_H or t["bottom_y"] < MIN_TARGET_BOTTOM_Y: continue

        center_penalty = abs(t["cx"] - 0.5)
        base_metric = (t["bottom_y"] * 100.0) - (center_penalty * 0.2)

        keep_bonus = 0
        if last_target and t["label"] == last_target["label"]:
            if abs(t["cx"] - last_target["cx"]) < TRACK_MATCH_CX:
                keep_bonus = 0.5

        t["final_metric"] = base_metric + keep_bonus
        candidates.append(t)

    if not candidates:
        last_target = None
        target_hold_count = 0
        return None

    candidates.sort(key=lambda x: x["final_metric"], reverse=True)
    best = candidates[0]

    if last_target and best["label"] == last_target["label"]:
        target_hold_count = min(target_hold_count + 1, TARGET_HOLD_MAX)
    else:
        target_hold_count = 0

    last_target = best
    return best


def draw_yellow_rois(img):
    for roi in YELLOW_ROIS:
        img.draw_rectangle(roi, color=(0, 255, 255), thickness=1)

def find_yellow_line(img):
    global yellow_count
    best_blob = None
    max_metric = -1
    #draw_yellow_rois(img)

    for roi in YELLOW_ROIS:
        blobs = img.find_blobs(
            YELLOW_THRESHOLDS, roi=roi,
            pixels_threshold=YELLOW_PIXELS_THRESH,
            area_threshold=YELLOW_AREA_THRESH, merge=True
        )
        for blob in blobs:
            if blob.w() < YELLOW_MIN_W or blob.h() < YELLOW_MIN_H: continue
            metric = blob.pixels() + blob.cy() * 5 + blob.w() * 2
            if metric > max_metric:
                max_metric = metric
                best_blob = blob

    current_found = False
    if best_blob:
        img.draw_rectangle(best_blob.rect(), color=(255, 255, 0), thickness=2)
        img.draw_cross(best_blob.cx(), best_blob.cy(), color=(255, 0, 0))
        if best_blob.cy() > YELLOW_STOP_LINE_Y:
            current_found = True

    yellow_count = yellow_count + 1 if current_found else 0
    return yellow_count >= YELLOW_COUNT_OK


def send_target(target):
    if target is None:
        uart.write("N\n")
        return

    if target["label"] == 5:
        msg = "T,{},{},{},{},{},{},{},{}\n".format(
            int(target["label"]), int(target["score"] * 1000),
            int(target["cx"] * 1000), int(target["cy"] * 1000),
            int(target["w"] * 1000), int(target["h"] * 1000),
            int(target["bottom_y"] * 1000), int(target.get("y_diff", 0))
        )
    else:
        msg = "T,{},{},{},{},{},{},{}\n".format(
            int(target["label"]), int(target["score"] * 1000),
            int(target["cx"] * 1000), int(target["cy"] * 1000),
            int(target["w"] * 1000), int(target["h"] * 1000),
            int(target["bottom_y"] * 1000)
        )
    uart.write(msg)


def send_stop():
    uart.write("STOP\n")


def refine_detection_with_color(img, obj):
    try:
        x1, y1, x2, y2, label, score = obj
        label = int(label)
        w, h = x2 - x1, y2 - y1

        px1, py1 = int((x1 - 0.1) * img.width()), int(y1 * img.height())
        pw, ph = int(w * img.width()), int(h * img.height())

        px1 = max(0, px1)
        py1 = max(0, py1)
        if px1 + pw > img.width(): pw = img.width() - px1
        if py1 + ph > img.height(): ph = img.height() - py1

        if pw <= 0 or ph <= 0: return None

        final_x, final_y, final_w, final_h = px1, py1, pw, ph
        roi = (0, py1, img.width(), ph)

        if label in THRESHOLDS:
            blobs = img.find_blobs(
                THRESHOLDS[label], roi=roi,
                pixels_threshold=COLOR_PIXELS_THRESH,
                area_threshold=COLOR_AREA_THRESH, merge=True
            )
            if blobs:
                max_blob = max(blobs, key=lambda b: b.w() * b.h())
                final_x, final_w = max_blob.x(), max_blob.w()
                if final_x < 0: final_x = 0
                if final_x + final_w > img.width(): final_w = img.width() - final_x

        norm_box = pixel_rect_to_norm(final_x, final_y, final_w, final_h, img.width(), img.height())
        return norm_box, label, score
    except Exception as e:
        print("REFINE ERR:", e)
        return None


while True:
    clock.tick()
    parse_command()

    img = sensor.snapshot()
    ctrl_detections = []

    #yellow_found = find_yellow_line(img)

    #if yellow_found:
        #send_stop()
        #print("SEND: STOP")
        #gc.collect()
        #continue

    # 识别绿色块
    if desired_class == 5:
        blobs = img.find_blobs(
            GREEN_THRESHOLDS,
            pixels_threshold=COLOR_PIXELS_THRESH,
            area_threshold=COLOR_AREA_THRESH, merge=True
        )
        if blobs:
            max_blob = max(blobs, key=lambda b: b.pixels())
            corners = max_blob.min_corners()
            sorted_corners = sorted(corners, key=lambda pt: pt[0])
            left_pts = sorted(sorted_corners[:2], key=lambda pt: pt[1])
            right_pts = sorted(sorted_corners[2:], key=lambda pt: pt[1])

            y_diff = left_pts[0][1] - right_pts[0][1]

            norm_box = pixel_rect_to_norm(
                max_blob.x(), max_blob.y(), max_blob.w(), max_blob.h(),
                img.width(), img.height()
            )
            if norm_box is not None:
                ctrl_detections.append((
                    norm_box[0], norm_box[1], norm_box[2], norm_box[3], 5, 1.0, y_diff
                ))
            img.draw_edges(corners, color=(0, 255, 0), thickness=1)

    else:
        # 常规模型识别
        for obj in tf.detect(net, img):
            try:
                x1, y1, x2, y2, label, score = obj
                if score < SCORE_THRESH: continue
                result = refine_detection_with_color(img, obj)
                if result is None: continue
                norm_box, label, score = result
                if norm_box is None: continue
                ctrl_detections.append((
                    norm_box[0], norm_box[1], norm_box[2], norm_box[3], label, score
                ))
            except Exception as e:
                print("DET ERR:", e)

    # 获得当前帧的原始最优解（带噪点的状态）
    raw_target = choose_best_target(ctrl_detections)
    raw_label = raw_target["label"] if raw_target is not None else -1

    # =========================================================
    # 【核心新增】时域防抖滤波状态机
    # =========================================================
    final_target = None

    if tracking_label != -1:
        # 情况A：当前已经锁定了一个正在跟踪的目标
        if raw_label == tracking_label:
            # 1. 目标继续完美匹配 -> 重置丢失计数，更新坐标
            missing_count = 0
            candidate_count = 0
            last_valid_target = raw_target
            final_target = raw_target
        elif raw_label == -1:
            # 2. 画面中啥也没有了（可能闪退了一帧）-> 累加丢失计数
            missing_count += 1
            if missing_count >= FILTER_LOSE_FRAMES:
                tracking_label = -1  # 彻底丢失，回到没跟踪状态
            else:
                final_target = last_valid_target  # 用旧数据顶替一下，假装还在
        else:
            # 3. 画面中看到了跟当前目标不一样的其他东西（可能是杂散颜色）
            missing_count += 1 # 对于正在跟踪的目标来说，相当于丢失了一帧
            if raw_label == candidate_label:
                candidate_count += 1
                if candidate_count >= FILTER_CONFIRM_FRAMES:
                    # 这个新东西连续出现了好几帧，说明不是干扰，篡位成功！
                    tracking_label = raw_label
                    last_valid_target = raw_target
                    final_target = raw_target
                    missing_count = 0
                else:
                    # 新东西还没稳定，我们继续假装还是旧目标
                    if missing_count >= FILTER_LOSE_FRAMES:
                        tracking_label = -1
                    else:
                        final_target = last_valid_target
            else:
                # 记录一下这个突然冒出来的新东西
                candidate_label = raw_label
                candidate_count = 1
                if missing_count >= FILTER_LOSE_FRAMES:
                    tracking_label = -1
                else:
                    final_target = last_valid_target
    else:
        # 情况B：当前没跟踪任何东西 (游离状态)
        if raw_label != -1:
            if raw_label == candidate_label:
                candidate_count += 1
                if candidate_count >= FILTER_CONFIRM_FRAMES:
                    # 这个新物体连续存在足够久，确认为真目标！
                    tracking_label = raw_label
                    last_valid_target = raw_target
                    final_target = raw_target
                    missing_count = 0
                else:
                    final_target = None  # 还在确认中，先不急着发
            else:
                candidate_label = raw_label
                candidate_count = 1
                final_target = None
        else:
            candidate_count = 0
            final_target = None

    # =========================================================

    # 发送经过滤波清洗过的最终决定 final_target
    send_target(final_target)

    # 画框和串口打印也替换为 final_target
    if final_target is not None:
        rect = safe_rect_from_norm(
            final_target["x1"], final_target["y1"],
            final_target["x2"], final_target["y2"],
            img.width(), img.height()
        )

        if rect is not None:
            box_color = (0, 255, 0) if final_target["label"] == 5 else (255, 0, 0)
            img.draw_rectangle(rect, color=box_color, thickness=2)

        if final_target["label"] == 5:
            print(
                "SEND (GREEN):", final_target["label"], round(final_target["score"], 3),
                "y_diff:", final_target.get("y_diff", 0), "CTRL:", len(ctrl_detections)
            )
        else:
            print(
                "SEND:", final_target["label"], round(final_target["score"], 3),
                round(final_target["cx"], 3), round(final_target["bottom_y"], 3),
                "CTRL:", len(ctrl_detections)
            )
    else:
        print("SEND: N", "CTRL:", len(ctrl_detections))

    gc.collect()

