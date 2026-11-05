# -*- coding: utf-8 -*-
import sensor
import gc

import config

from camera_init import init_camera
from protocol import MainboardProtocol
from model_detector import OriginalCorrectionDetector
from target_filter import TargetTemporalFilter
from yellow_line import YellowLineDetector
from yellow_line import YellowYawDetector
from vision_utils import norm_rect_to_pixel


clock = init_camera()

protocol = MainboardProtocol()
detector = OriginalCorrectionDetector()
target_filter = TargetTemporalFilter()
yellow_detector = YellowLineDetector()
yellow_yaw_detector = YellowYawDetector()

desired_class = config.START_DESIRED_CLASS
lower_roi_only = False

# V,0 / V,1 控制全部视觉识别。
recognition_enabled = True

# RED,0 / RED,1 控制是否接受模型类别 0。
red_sandbag_enabled = True

# IMU,1 / IMU,0 控制黄线航向测量模式。
yaw_line_mode = False

disabled_frame_count = 0


print("================================")
print("VISION LOCK + LOWER ROI READY")
print("TARGET CLASS:", desired_class)
print("LOWER ROI:", lower_roi_only)
print("RECOGNITION:", recognition_enabled)
print("RED SANDBAG:", red_sandbag_enabled)
print("YAW LINE MODE:", yaw_line_mode)
print("UART OUTPUT: N / T / STOP / Y / YN")
print("================================")


def reset_tracking():
    detector.reset()
    target_filter.reset()
    yellow_detector.reset()
    yellow_yaw_detector.reset()


def set_target(new_class):
    global desired_class

    if new_class != -1:
        if (
            new_class < 0
            or new_class >= len(config.LABELS)
        ):
            return

    # 搜索结束后，主板会把 -1 收窄成当前识别到的类别。
    # 若锁定目标本来就是该类别，则保留锁定，避免重新确认期间短暂 N。
    keep_locked_target = False

    if (
        new_class != -1
        and target_filter.last_valid is not None
        and target_filter.last_valid["label"] == new_class
    ):
        keep_locked_target = True

    desired_class = new_class

    if keep_locked_target:
        target_filter.candidate = None
        target_filter.candidate_count = 0
    else:
        reset_tracking()

    print(
        ">>> Target Changed To:",
        desired_class,
        "KEEP_LOCK:",
        keep_locked_target
    )


def set_lower_roi(enabled):
    global lower_roi_only

    lower_roi_only = bool(enabled)
    target_filter.set_near_mode(
        lower_roi_only
    )

    # 不返回 ACK，只在电脑终端打印。
    print(">>> LOWER ROI:", lower_roi_only)


def set_recognition(enabled):
    global recognition_enabled

    enabled = bool(enabled)

    if recognition_enabled == enabled:
        return

    recognition_enabled = enabled
    reset_tracking()

    # 不向主板返回 ACK，只在电脑终端打印状态。
    print(">>> RECOGNITION:", recognition_enabled)


def set_red_sandbag_enabled(enabled):
    global red_sandbag_enabled

    enabled = bool(enabled)

    if red_sandbag_enabled == enabled:
        return

    red_sandbag_enabled = enabled

    # 切换类别屏蔽状态后清空旧锁定，避免类别 0 被继续保持。
    reset_tracking()

    # 不向主板返回 ACK，只在电脑终端打印状态。
    print(">>> RED SANDBAG:", red_sandbag_enabled)


def set_yaw_line_mode(enabled):
    global yaw_line_mode

    enabled = bool(enabled)

    if yaw_line_mode == enabled:
        return

    yaw_line_mode = enabled
    reset_tracking()

    # 不返回 ACK，只在电脑终端打印。
    print(">>> YAW LINE MODE:", yaw_line_mode)


def handle_commands():
    for command in protocol.poll():
        command_type = command.get("type")

        if command_type == "target":
            set_target(
                command.get("value")
            )

        elif command_type == "roi":
            set_lower_roi(
                command.get("lower_only", False)
            )

        elif command_type == "recognition":
            set_recognition(
                command.get("enabled", True)
            )

        elif command_type == "red_sandbag":
            set_red_sandbag_enabled(
                command.get("enabled", True)
            )

        elif command_type == "yaw_line":
            set_yaw_line_mode(
                command.get("enabled", False)
            )


def draw_final_target(img, target):
    rect = norm_rect_to_pixel(
        target["x1"],
        target["y1"],
        target["x2"],
        target["y2"],
        img.width(),
        img.height()
    )

    if rect is not None:
        img.draw_rectangle(
            rect,
            color=(255, 0, 0),
            thickness=2
        )

    img.draw_cross(
        int(target["cx"] * img.width()),
        int(target["cy"] * img.height()),
        color=(0, 255, 0)
    )


def filter_lower_candidates(candidates):
    if not lower_roi_only:
        return candidates

    result = []

    for candidate in candidates:
        if (
            candidate["bottom_y"]
            >= config.LOWER_ROI_MIN_BOTTOM_Y
        ):
            result.append(candidate)

    return result


while True:
    clock.tick()

    handle_commands()

    img = sensor.snapshot()

    # =====================================================
    # IMU,1：黄线航向标定专用模式
    #
    # 该模式优先于 V,0，便于普通识别关闭时仍可校正 IMU。
    # 不执行物体模型、不发送 N/T/STOP，只发送 Y 或 YN。
    # =====================================================
    if yaw_line_mode:
        yaw_result = yellow_yaw_detector.detect(
            img
        )

        if yaw_result is None:
            protocol.send_yaw_line_none()

            if config.PRINT_DEBUG:
                print("YAW LINE: NONE")
        else:
            protocol.send_yaw_line(
                yaw_result
            )

            if config.PRINT_DEBUG:
                print(
                    "YAW LINE:",
                    round(
                        yaw_result["angle_deg"],
                        2
                    ),
                    "QUALITY:",
                    yaw_result["quality"],
                    "CY:",
                    round(
                        yaw_result["cy"],
                        3
                    ),
                    "SPREAD:",
                    round(
                        yaw_result["spread_deg"],
                        2
                    )
                )

        gc.collect()
        continue

    # =====================================================
    # V,0：关闭全部视觉识别
    #
    # 仍然拍照并轮询 UART，保证可以随时接收 V,1；
    # 但不执行黄线检测、模型推理，也不向主板发送 N/T/STOP。
    # =====================================================
    if not recognition_enabled:
        disabled_frame_count += 1

        if (
            config.PRINT_DEBUG
            and disabled_frame_count
            >= config.IDLE_PRINT_INTERVAL_FRAMES
        ):
            disabled_frame_count = 0
            print(
                "VISION DISABLED FPS:",
                round(clock.fps(), 1)
            )

        gc.collect()
        continue

    disabled_frame_count = 0

    # =====================================================
    # 黄色边线优先
    # =====================================================
    if config.ENABLE_YELLOW_LINE:
        if yellow_detector.detect(img):
            protocol.send_stop()

            if config.PRINT_DEBUG:
                print("SEND: STOP")

            gc.collect()
            continue

    # =====================================================
    # 模型 + 原版色块二次纠正
    # =====================================================
    ignored_labels = None

    if not red_sandbag_enabled:
        ignored_labels = (0,)

    raw_target, candidates = detector.detect(
        img,
        desired_class=desired_class,
        ignored_labels=ignored_labels
    )

    candidates = filter_lower_candidates(
        candidates
    )

    if candidates:
        raw_target = candidates[0]
    else:
        raw_target = None

    final_target = target_filter.update(
        raw_target,
        candidates
    )

    if final_target is None:
        protocol.send_no_target()

        if config.PRINT_DEBUG:
            print(
                "SEND: N",
                "LOWER:", lower_roi_only,
                "RED:", red_sandbag_enabled,
                "CAND:", len(candidates),
                "FPS:", round(clock.fps(), 1)
            )

    else:
        protocol.send_target(
            final_target
        )

        if config.DRAW_DEBUG:
            draw_final_target(
                img,
                final_target
            )

        if config.PRINT_DEBUG:
            print(
                "SEND:",
                final_target["label"],
                config.LABELS[
                    final_target["label"]
                ],
                round(
                    final_target["score"],
                    3
                ),
                "cx:",
                round(
                    final_target["cx"],
                    3
                ),
                "bottom:",
                round(
                    final_target["bottom_y"],
                    3
                ),
                "LOWER:", lower_roi_only,
                "RED:", red_sandbag_enabled,
                "CORRECTED:",
                final_target["corrected"],
                "CAND:",
                len(candidates),
                "FPS:",
                round(clock.fps(), 1)
            )

    gc.collect()
