# -*- coding: utf-8 -*-
"""
比赛现场独立调试：

    不使用主板命令
    不初始化 UART
    运行后立即执行模型和原版色块二次纠正

显示：
    黄色框：模型原始框
    蓝色框：原版整行颜色搜索 ROI
    绿色框：被选中的最大色块
    红色框：二次纠正后的最终框
"""

import sensor
import gc

import config

from camera_init import init_camera
from model_detector import OriginalCorrectionDetector
from target_filter import TargetTemporalFilter
from vision_utils import norm_rect_to_pixel


DEBUG_DESIRED_CLASS = -1
DEBUG_SCORE_THRESH = 0.20

# 是否显示整行颜色搜索区域。
DRAW_COLOR_SEARCH_ROI = True

# 是否显示原始模型框。
DRAW_RAW_MODEL_BOX = True

# 是否显示选中的最大色块。
DRAW_SELECTED_BLOB = True


clock = init_camera()
detector = OriginalCorrectionDetector()
target_filter = TargetTemporalFilter()

frame_index = 0


def draw_norm_rect(
    img,
    x1,
    y1,
    x2,
    y2,
    color
):
    rect = norm_rect_to_pixel(
        x1,
        y1,
        x2,
        y2,
        img.width(),
        img.height()
    )

    if rect is not None:
        img.draw_rectangle(
            rect,
            color=color,
            thickness=2
        )


print("================================")
print("ORIGINAL CORRECTION DEBUG READY")
print("NO MAINBOARD REQUIRED")
print("CLASS:", DEBUG_DESIRED_CLASS)
print("SCORE:", DEBUG_SCORE_THRESH)
print("================================")


while True:
    clock.tick()
    frame_index += 1

    img = sensor.snapshot()

    raw_best, candidates = detector.detect(
        img,
        desired_class=DEBUG_DESIRED_CLASS,
        score_threshold=DEBUG_SCORE_THRESH
    )

    final_target = target_filter.update(
        raw_best
    )

    for candidate in candidates:
        if DRAW_RAW_MODEL_BOX:
            draw_norm_rect(
                img,
                candidate["raw_x1"],
                candidate["raw_y1"],
                candidate["raw_x2"],
                candidate["raw_y2"],
                color=(255, 255, 0)
            )

        if DRAW_COLOR_SEARCH_ROI:
            img.draw_rectangle(
                candidate["color_roi"],
                color=(0, 255, 255),
                thickness=1
            )

        if (
            DRAW_SELECTED_BLOB
            and candidate[
                "selected_blob_rect"
            ] is not None
        ):
            img.draw_rectangle(
                candidate[
                    "selected_blob_rect"
                ],
                color=(0, 255, 0),
                thickness=2
            )

    if final_target is not None:
        draw_norm_rect(
            img,
            final_target["x1"],
            final_target["y1"],
            final_target["x2"],
            final_target["y2"],
            color=(255, 0, 0)
        )

    print(
        "FRAME:",
        frame_index,
        "COUNT:",
        len(candidates),
        "FPS:",
        round(clock.fps(), 1)
    )

    for index in range(len(candidates)):
        item = candidates[index]

        print(
            "  DET",
            index,
            "label:",
            item["label"],
            config.LABELS[item["label"]],
            "score:",
            round(item["score"], 3),
            "corrected:",
            item["corrected"],
            "raw_box:",
            round(item["raw_x1"], 3),
            round(item["raw_y1"], 3),
            round(item["raw_x2"], 3),
            round(item["raw_y2"], 3),
            "final_box:",
            round(item["x1"], 3),
            round(item["y1"], 3),
            round(item["x2"], 3),
            round(item["y2"], 3)
        )

    if final_target is None:
        print("  FINAL: NONE")
    else:
        print(
            "  FINAL:",
            final_target["label"],
            config.LABELS[
                final_target["label"]
            ],
            round(final_target["score"], 3),
            "corrected:",
            final_target["corrected"],
            "cx:",
            round(final_target["cx"], 3),
            "bottom:",
            round(
                final_target["bottom_y"],
                3
            )
        )

    gc.collect()
