# -*- coding: utf-8 -*-
import tf

import config

from vision_utils import clamp01
from vision_utils import pixel_rect_to_norm
from vision_utils import build_target


class OriginalCorrectionDetector:
    """
    恢复原程序的检测流程：

        tf.detect()
            ↓
        模型框左移 0.1 归一化宽度
            ↓
        在模型框同一高度的整幅图像横条中 find_blobs()
            ↓
        选面积最大的色块
            ↓
        用色块 x、w 替换模型框 x、w
            ↓
        y、h 继续使用模型框
    """

    def __init__(self):
        print("LOADING MODEL:", config.MODEL_PATH)

        self.net = tf.load(
            config.MODEL_PATH
        )

        self.last_target = None

        print("MODEL LOADED")

    def reset(self):
        self.last_target = None

    def _original_refine(self, img, obj):
        x1, y1, x2, y2, label, score = obj[:6]

        label = int(label)
        score = float(score)

        width_norm = float(x2) - float(x1)
        height_norm = float(y2) - float(y1)

        # 原代码的模型框像素转换。
        pixel_x = int(
            (
                float(x1)
                - config.ORIGINAL_X_OFFSET
            )
            * img.width()
        )
        pixel_y = int(
            float(y1)
            * img.height()
        )
        pixel_w = int(
            width_norm
            * img.width()
        )
        pixel_h = int(
            height_norm
            * img.height()
        )

        pixel_x = max(0, pixel_x)
        pixel_y = max(0, pixel_y)

        if pixel_x + pixel_w > img.width():
            pixel_w = img.width() - pixel_x

        if pixel_y + pixel_h > img.height():
            pixel_h = img.height() - pixel_y

        if pixel_w <= 0 or pixel_h <= 0:
            return None

        final_x = pixel_x
        final_y = pixel_y
        final_w = pixel_w
        final_h = pixel_h

        corrected = False
        selected_blob_rect = None

        # 原版模式：整幅图像同一高度的横条 ROI。
        color_roi = (
            0,
            pixel_y,
            img.width(),
            pixel_h
        )

        if (
            config.ENABLE_ORIGINAL_COLOR_CORRECTION
            and label in config.THRESHOLDS
        ):
            blobs = img.find_blobs(
                config.THRESHOLDS[label],
                roi=color_roi,
                pixels_threshold=(
                    config.COLOR_PIXELS_THRESH
                ),
                area_threshold=(
                    config.COLOR_AREA_THRESH
                ),
                merge=True
            )

            if blobs:
                max_blob = max(
                    blobs,
                    key=lambda blob: (
                        blob.w() * blob.h()
                    )
                )

                final_x = max_blob.x()
                final_w = max_blob.w()

                if final_x < 0:
                    final_x = 0

                if final_x + final_w > img.width():
                    final_w = (
                        img.width() - final_x
                    )

                corrected = True
                selected_blob_rect = (
                    max_blob.rect()
                )

        norm_box = pixel_rect_to_norm(
            final_x,
            final_y,
            final_w,
            final_h,
            img.width(),
            img.height()
        )

        if norm_box is None:
            return None

        target = build_target(
            norm_box[0],
            norm_box[1],
            norm_box[2],
            norm_box[3],
            label,
            score
        )

        # 供 competition_debug.py 显示原始模型框和色块。
        target["raw_x1"] = clamp01(float(x1))
        target["raw_y1"] = clamp01(float(y1))
        target["raw_x2"] = clamp01(float(x2))
        target["raw_y2"] = clamp01(float(y2))

        target["corrected"] = corrected
        target["color_roi"] = color_roi
        target["selected_blob_rect"] = (
            selected_blob_rect
        )

        return target

    def _metric(self, target):
        center_penalty = abs(
            target["cx"] - 0.5
        )

        metric = (
            target["bottom_y"]
            * config.TARGET_BOTTOM_WEIGHT
            - center_penalty
            * config.TARGET_CENTER_WEIGHT
        )

        if (
            self.last_target is not None
            and target["label"]
            == self.last_target["label"]
            and abs(
                target["cx"]
                - self.last_target["cx"]
            ) < config.TRACK_MATCH_CX
        ):
            metric += config.TARGET_KEEP_BONUS

        return metric

    def detect(
        self,
        img,
        desired_class=-1,
        score_threshold=None,
        ignored_labels=None
    ):
        if score_threshold is None:
            score_threshold = (
                config.SCORE_THRESH
            )

        candidates = []

        try:
            raw_results = tf.detect(
                self.net,
                img
            )
        except Exception as error:
            print("TF DETECT ERR:", error)
            return None, candidates

        for obj in raw_results:
            try:
                if len(obj) < 6:
                    continue

                label = int(obj[4])
                score = float(obj[5])

                if score < score_threshold:
                    continue

                if (
                    label < 0
                    or label >= len(config.LABELS)
                ):
                    continue

                if (
                    desired_class != -1
                    and label != desired_class
                ):
                    continue

                # RED,0 时在进入色块二次纠正前直接丢弃类别 0，
                # 防止红砖误识别为红沙包后继续进入控制流程。
                if (
                    ignored_labels is not None
                    and label in ignored_labels
                ):
                    continue

                target = self._original_refine(
                    img,
                    obj
                )

                if target is None:
                    continue

                if target["h"] < config.MIN_TARGET_H:
                    continue

                if (
                    target["bottom_y"]
                    < config.MIN_TARGET_BOTTOM_Y
                ):
                    continue

                target["metric"] = self._metric(
                    target
                )

                candidates.append(target)

            except Exception as error:
                print("DET ITEM ERR:", error)

        if not candidates:
            self.last_target = None
            return None, candidates

        candidates.sort(
            key=lambda item: item["metric"],
            reverse=True
        )

        best = candidates[0]
        self.last_target = best

        return best, candidates
