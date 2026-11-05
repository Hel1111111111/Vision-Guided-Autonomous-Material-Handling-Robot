# -*- coding: utf-8 -*-
import math

import config


class YellowLineDetector:
    def __init__(self):
        self.count = 0

    def reset(self):
        self.count = 0

    def detect(self, img):
        best_blob = None
        best_metric = -1

        for roi in config.YELLOW_ROIS:
            if config.DRAW_YELLOW_ROI:
                img.draw_rectangle(
                    roi,
                    color=(0, 255, 255),
                    thickness=1
                )

            blobs = img.find_blobs(
                config.YELLOW_THRESHOLDS,
                roi=roi,
                pixels_threshold=(
                    config.YELLOW_PIXELS_THRESH
                ),
                area_threshold=(
                    config.YELLOW_AREA_THRESH
                ),
                merge=True
            )

            for blob in blobs:
                if blob.w() < config.YELLOW_MIN_W:
                    continue

                if blob.h() < config.YELLOW_MIN_H:
                    continue

                metric = (
                    blob.pixels()
                    + blob.cy() * 5
                    + blob.w() * 2
                )

                if metric > best_metric:
                    best_metric = metric
                    best_blob = blob

        found = False

        if best_blob is not None:
            if config.DRAW_DEBUG:
                img.draw_rectangle(
                    best_blob.rect(),
                    color=(255, 255, 0),
                    thickness=2
                )
                img.draw_cross(
                    best_blob.cx(),
                    best_blob.cy(),
                    color=(255, 0, 0)
                )

            if (
                best_blob.cy()
                > config.YELLOW_STOP_LINE_Y
            ):
                found = True

        if found:
            self.count += 1
        else:
            self.count = 0

        return (
            self.count
            >= config.YELLOW_CONFIRM_FRAMES
        )


class YellowYawDetector:
    """
    使用场地 5 cm 黄线估计其在图像中的倾斜角。

    流程：
        1. 在下半幅 ROI 查找细长黄色色块；
        2. 确认黄色长条具有足够宽度和像素支持；
        3. 对 ROI 中全部黄色像素执行直线回归；
        4. 用端点计算相对画面水平线的角度；
        5. 连续多帧中值和极差检查后输出稳定角度。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.angle_history = []
        self.stable_count = 0
        self.last_result = None

    @staticmethod
    def _object_value(obj, name, index=None):
        """
        同时兼容旧固件的 obj.x1() 和新接口的 obj.x1 属性。
        """
        try:
            value = getattr(obj, name)

            if callable(value):
                value = value()

            return value
        except Exception:
            if index is not None:
                return obj[index]
            raise

    @staticmethod
    def _normalize_line_angle(angle_deg):
        """
        直线没有前后方向，将角度归一化到 [-90, 90]。
        """
        while angle_deg > 90.0:
            angle_deg -= 180.0

        while angle_deg < -90.0:
            angle_deg += 180.0

        return angle_deg

    @staticmethod
    def _median(values):
        ordered = sorted(values)
        count = len(ordered)

        if count == 0:
            return 0.0

        middle = count // 2

        if count & 1:
            return float(ordered[middle])

        return (
            float(ordered[middle - 1])
            + float(ordered[middle])
        ) * 0.5

    def _find_boundary_support(self, img):
        blobs = img.find_blobs(
            config.YELLOW_THRESHOLDS,
            roi=config.YAW_LINE_ROI,
            pixels_threshold=(
                config.YAW_LINE_BLOB_PIXELS_THRESH
            ),
            area_threshold=(
                config.YAW_LINE_BLOB_AREA_THRESH
            ),
            merge=True
        )

        supports = []
        support_width = 0
        support_pixels = 0
        weighted_cy_sum = 0

        for blob in blobs:
            width = int(
                self._object_value(blob, "w", 2)
            )
            height = int(
                self._object_value(blob, "h", 3)
            )
            pixels = int(
                self._object_value(blob, "pixels", 4)
            )

            if width < config.YAW_LINE_MIN_BLOB_W:
                continue

            if height < config.YAW_LINE_MIN_BLOB_H:
                continue

            aspect = float(width) / max(1, height)

            if (
                aspect
                < config.YAW_LINE_MIN_BLOB_ASPECT
            ):
                continue

            cy = int(
                self._object_value(blob, "cy", 6)
            )

            supports.append(blob)
            support_width += width
            support_pixels += pixels
            weighted_cy_sum += cy * pixels

        if (
            support_width
            < config.YAW_LINE_MIN_SUPPORT_WIDTH
        ):
            return None

        if (
            support_pixels
            < config.YAW_LINE_MIN_SUPPORT_PIXELS
        ):
            return None

        if support_pixels <= 0:
            return None

        return {
            "blobs": supports,
            "support_width": support_width,
            "support_pixels": support_pixels,
            "cy": (
                float(weighted_cy_sum)
                / float(support_pixels)
            ),
        }

    def _get_regression_line(self, img):
        try:
            return img.get_regression(
                config.YELLOW_THRESHOLDS,
                roi=config.YAW_LINE_ROI,
                pixels_threshold=(
                    config.YAW_LINE_REG_PIXELS_THRESH
                ),
                area_threshold=(
                    config.YAW_LINE_REG_AREA_THRESH
                )
            )
        except TypeError:
            # 兼容不支持全部关键字参数的旧固件。
            try:
                return img.get_regression(
                    config.YELLOW_THRESHOLDS,
                    roi=config.YAW_LINE_ROI
                )
            except Exception as error:
                print("YAW REGRESSION ERR:", error)
                return None
        except Exception as error:
            print("YAW REGRESSION ERR:", error)
            return None

    def _raw_measurement(self, img):
        support = self._find_boundary_support(img)

        if support is None:
            return None

        line = self._get_regression_line(img)

        if line is None:
            return None

        try:
            x1 = int(
                self._object_value(line, "x1", 0)
            )
            y1 = int(
                self._object_value(line, "y1", 1)
            )
            x2 = int(
                self._object_value(line, "x2", 2)
            )
            y2 = int(
                self._object_value(line, "y2", 3)
            )
            length = int(
                self._object_value(line, "length", 4)
            )
            magnitude = int(
                self._object_value(
                    line,
                    "magnitude",
                    5
                )
            )
        except Exception as error:
            print("YAW LINE VALUE ERR:", error)
            return None

        if length < config.YAW_LINE_MIN_LENGTH:
            return None

        if (
            magnitude
            < config.YAW_LINE_MIN_MAGNITUDE
        ):
            return None

        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return None

        # 图像 y 轴向下，因此该符号是“图像坐标系角度”。
        angle_deg = math.degrees(
            math.atan2(dy, dx)
        )
        angle_deg = self._normalize_line_angle(
            angle_deg
        )

        if (
            abs(angle_deg)
            > config.YAW_LINE_MAX_ABS_ANGLE_DEG
        ):
            return None

        width_score = min(
            1.0,
            float(support["support_width"])
            / 220.0
        )
        length_score = min(
            1.0,
            float(length) / 240.0
        )
        pixel_score = min(
            1.0,
            float(support["support_pixels"])
            / 1200.0
        )

        quality = int(
            (
                width_score * 0.35
                + length_score * 0.45
                + pixel_score * 0.20
            )
            * 1000.0
        )

        if config.DRAW_YAW_LINE_DEBUG:
            for blob in support["blobs"]:
                try:
                    rect = self._object_value(
                        blob,
                        "rect",
                        None
                    )
                    img.draw_rectangle(
                        rect,
                        color=(0, 255, 0),
                        thickness=1
                    )
                except Exception:
                    pass

            img.draw_line(
                (x1, y1, x2, y2),
                color=(255, 255, 255),
                thickness=2
            )

        return {
            "angle_deg": angle_deg,
            "quality": quality,
            "cy": (
                support["cy"]
                / float(img.height())
            ),
            "line": (x1, y1, x2, y2),
            "length": length,
            "magnitude": magnitude,
        }

    def detect(self, img):
        raw = self._raw_measurement(img)

        if raw is None:
            self.angle_history = []
            self.stable_count = 0
            self.last_result = None
            return None

        self.angle_history.append(
            raw["angle_deg"]
        )

        history_limit = (
            config.YAW_LINE_HISTORY_SIZE
        )

        if (
            len(self.angle_history)
            > history_limit
        ):
            self.angle_history.pop(0)

        filtered_angle = self._median(
            self.angle_history
        )

        spread = (
            max(self.angle_history)
            - min(self.angle_history)
        )

        if (
            len(self.angle_history)
            >= config.YAW_LINE_CONFIRM_FRAMES
            and spread
            <= config.YAW_LINE_STABLE_SPREAD_DEG
        ):
            self.stable_count += 1
        else:
            self.stable_count = 0

        if (
            self.stable_count
            < config.YAW_LINE_CONFIRM_FRAMES
        ):
            return None

        result = dict(raw)
        result["angle_deg"] = filtered_angle
        result["spread_deg"] = spread
        result["stable_count"] = (
            self.stable_count
        )

        self.last_result = result
        return result
