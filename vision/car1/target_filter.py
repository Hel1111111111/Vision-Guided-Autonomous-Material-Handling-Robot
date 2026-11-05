# -*- coding: utf-8 -*-
import config


class TargetTemporalFilter:
    """
    目标锁定滤波：

    1. 新目标需要连续确认；
    2. 已锁定后，短暂 N 或错误目标继续输出上一目标；
    3. 突然跳到另一个目标时，必须连续多帧确认才切换；
    4. 下部 ROI 模式下，拒绝明显“变远、变小”的同类目标，
       避免前排物体消失后锁到后排物体。
    """

    def __init__(self):
        self.near_mode = False
        self.reset()

    def reset(self):
        self.tracking = False
        self.last_valid = None

        self.candidate = None
        self.candidate_count = 0

        self.missing_count = 0

    def set_near_mode(self, enabled):
        self.near_mode = bool(enabled)

        # 切换 ROI 时保留已锁定目标，但清掉尚未确认的跳变候选。
        self.candidate = None
        self.candidate_count = 0

    @staticmethod
    def _size_ratio(a, b, key):
        av = a[key]
        bv = b[key]

        if av <= 0.0001 or bv <= 0.0001:
            return 0.0

        if av < bv:
            return av / bv

        return bv / av

    def _same_identity(self, incoming, reference):
        if incoming is None or reference is None:
            return False

        if incoming["label"] != reference["label"]:
            return False

        if (
            abs(incoming["cx"] - reference["cx"])
            > config.FILTER_MATCH_CX
        ):
            return False

        if (
            abs(
                incoming["bottom_y"]
                - reference["bottom_y"]
            )
            > config.FILTER_MATCH_BOTTOM_Y
        ):
            return False

        if (
            self._size_ratio(incoming, reference, "w")
            < config.FILTER_MIN_SIZE_RATIO
        ):
            return False

        if (
            self._size_ratio(incoming, reference, "h")
            < config.FILTER_MIN_SIZE_RATIO
        ):
            return False

        if self.near_mode:
            # 接近阶段突然看到更高、更小的同类物体，通常是后排目标。
            if (
                incoming["bottom_y"]
                < reference["bottom_y"]
                - config.NEAR_MAX_BACKWARD_BOTTOM_Y
            ):
                return False

            if (
                incoming["h"]
                < reference["h"]
                * config.NEAR_MIN_HEIGHT_RATIO
            ):
                return False

        return True

    def _same_candidate(self, a, b):
        if a is None or b is None:
            return False

        if a["label"] != b["label"]:
            return False

        if (
            abs(a["cx"] - b["cx"])
            > config.FILTER_MATCH_CX
        ):
            return False

        if (
            abs(a["bottom_y"] - b["bottom_y"])
            > config.FILTER_MATCH_BOTTOM_Y
        ):
            return False

        return True

    @staticmethod
    def _candidate_cost(candidate, reference):
        return (
            abs(candidate["cx"] - reference["cx"])
            + abs(
                candidate["bottom_y"]
                - reference["bottom_y"]
            )
            + 0.5 * abs(candidate["w"] - reference["w"])
            + 0.5 * abs(candidate["h"] - reference["h"])
        )

    def _find_matching_candidate(self, candidates):
        if self.last_valid is None:
            return None

        matches = []

        for candidate in candidates:
            if self._same_identity(
                candidate,
                self.last_valid
            ):
                matches.append(candidate)

        if not matches:
            return None

        matches.sort(
            key=lambda item: self._candidate_cost(
                item,
                self.last_valid
            )
        )

        return matches[0]

    @staticmethod
    def _blend(old, new):
        alpha = config.FILTER_SMOOTH_ALPHA
        result = dict(new)

        for key in (
            "score",
            "x1", "y1", "x2", "y2",
            "cx", "cy", "w", "h", "bottom_y"
        ):
            result[key] = (
                old[key] * (1.0 - alpha)
                + new[key] * alpha
            )

        return result

    def _start_candidate(self, candidate):
        if self._same_candidate(candidate, self.candidate):
            self.candidate_count += 1
        else:
            self.candidate = candidate
            self.candidate_count = 1

    def update(self, raw_target=None, candidates=None):
        if candidates is None:
            if raw_target is None:
                candidates = []
            else:
                candidates = [raw_target]

        # =====================================================
        # 已锁定目标
        # =====================================================
        if self.tracking:
            matched = self._find_matching_candidate(
                candidates
            )

            if matched is not None:
                self.last_valid = self._blend(
                    self.last_valid,
                    matched
                )

                self.missing_count = 0
                self.candidate = None
                self.candidate_count = 0

                return self.last_valid

            # 没有与原目标匹配的候选：先保留旧数据。
            self.missing_count += 1

            # 下部 ROI 模式下不允许直接切到远处同类目标。
            # 即使画面中有其他候选，也只把它当作干扰。
            if not self.near_mode and candidates:
                new_candidate = candidates[0]
                self._start_candidate(new_candidate)

                if (
                    self.candidate_count
                    >= config.FILTER_SWITCH_CONFIRM_FRAMES
                ):
                    self.last_valid = new_candidate
                    self.candidate = None
                    self.candidate_count = 0
                    self.missing_count = 0
                    return self.last_valid

            if self.missing_count < config.FILTER_LOSE_FRAMES:
                return self.last_valid

            # 连续丢失超过阈值后才输出 N。
            self.reset()
            return None

        # =====================================================
        # 尚未锁定
        # =====================================================
        if not candidates:
            self.candidate = None
            self.candidate_count = 0
            return None

        new_candidate = candidates[0]
        self._start_candidate(new_candidate)

        if (
            self.candidate_count
            >= config.FILTER_CONFIRM_FRAMES
        ):
            self.tracking = True
            self.last_valid = new_candidate
            self.candidate = None
            self.candidate_count = 0
            self.missing_count = 0

            return self.last_valid

        return None
