# vision.py
# 摄像头串口模块：保留原来的 C,n / N / T / STOP 协议，
# 额外增加主板->摄像头命令：
# R,1 / R,0：下部 ROI 开关；
# V,0 / V,1：视觉识别总开关；
# RED,0 / RED,1：红沙包类别开关；
# IMU,1 / IMU,0：黄线航向测量模式。
# 摄像头无需返回 ACK。

from machine import UART
import time
from config import VisionConfig


class VisionManager:
    def __init__(self):
        self.uart = UART(
            VisionConfig.UART_ID,
            VisionConfig.UART_BAUD
        )

        self.last_vision_line = 'NO_DATA'
        self.stop_signal = 0

        self.target = self._empty_target()
        self.last_good_rx_ms = time.ticks_ms()
        self.missing_since_ms = None

        # 新目标/跳变目标确认状态。
        self.candidate = None
        self.candidate_count = 0

        # 当前目标连续稳定计数。
        self.stable_count = 0

        # 主板本地保存的摄像头工作状态。
        self.recognition_enabled = True
        self.red_sandbag_enabled = True

        # 黄线航向测量状态。
        self.yaw_line_mode = False
        self.yaw_line_valid = False
        self.yaw_line_angle_deg = 0.0
        self.yaw_line_quality = 0
        self.yaw_line_cy = 0.0
        self.yaw_line_history = []
        self.yaw_line_stable_count = 0
        self.yaw_line_last_rx_ms = time.ticks_ms()

    @staticmethod
    def _empty_target():
        return {
            'valid': False,
            'class_id': -1,
            'class_name': 'none',
            'score': 0.0,
            'cx': 0.0,
            'cy': 0.0,
            'w': 0.0,
            'h': 0.0,
            'bottom_y': 0.0,
        }

    @staticmethod
    def _clamp01(value):
        if value < 0.0:
            return 0.0
        if value > 1.0:
            return 1.0
        return value

    def reset_target(self):
        self.target = self._empty_target()
        self.candidate = None
        self.candidate_count = 0
        self.stable_count = 0
        self.missing_since_ms = None


    def reset_yaw_line(self):
        self.yaw_line_valid = False
        self.yaw_line_angle_deg = 0.0
        self.yaw_line_quality = 0
        self.yaw_line_cy = 0.0
        self.yaw_line_history = []
        self.yaw_line_stable_count = 0
        self.yaw_line_last_rx_ms = time.ticks_ms()

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

    def _handle_yaw_line_packet(
        self,
        angle_deg,
        quality,
        cy
    ):
        if quality < VisionConfig.YAW_LINE_MIN_QUALITY:
            return

        if angle_deg < -90.0 or angle_deg > 90.0:
            return

        if cy < 0.0 or cy > 1.0:
            return

        self.yaw_line_history.append(
            float(angle_deg)
        )

        if (
            len(self.yaw_line_history)
            > VisionConfig.YAW_LINE_HISTORY_SIZE
        ):
            self.yaw_line_history.pop(0)

        filtered_angle = self._median(
            self.yaw_line_history
        )

        spread = (
            max(self.yaw_line_history)
            - min(self.yaw_line_history)
        )

        self.yaw_line_angle_deg = filtered_angle
        self.yaw_line_quality = int(quality)
        self.yaw_line_cy = float(cy)
        self.yaw_line_last_rx_ms = time.ticks_ms()

        if (
            len(self.yaw_line_history)
            >= VisionConfig.YAW_LINE_STABLE_SAMPLES
            and spread
            <= VisionConfig.YAW_LINE_STABLE_SPREAD_DEG
        ):
            self.yaw_line_stable_count += 1
            self.yaw_line_valid = True
        else:
            self.yaw_line_stable_count = 0
            self.yaw_line_valid = False

    def _handle_yaw_line_none(self):
        # 单帧 YN 不立即清空，持续丢失由超时机制处理。
        pass

    def yaw_line_timeout(self):
        if not self.yaw_line_valid:
            return True

        elapsed = time.ticks_diff(
            time.ticks_ms(),
            self.yaw_line_last_rx_ms
        )

        return (
            elapsed
            > VisionConfig.YAW_LINE_TIMEOUT_MS
        )

    def is_yaw_line_valid(self):
        return (
            self.yaw_line_valid
            and not self.yaw_line_timeout()
        )

    def is_yaw_line_stable(self):
        return (
            self.is_yaw_line_valid()
            and self.yaw_line_stable_count
            >= VisionConfig.YAW_LINE_STABLE_SAMPLES
        )

    def get_yaw_line_angle(self):
        if not self.is_yaw_line_valid():
            return None

        return self.yaw_line_angle_deg

    def get_yaw_line_quality(self):
        if not self.is_yaw_line_valid():
            return 0

        return self.yaw_line_quality

    def get_yaw_line_cy(self):
        if not self.is_yaw_line_valid():
            return 0.0

        return self.yaw_line_cy

    def _valid_packet_values(self, packet):
        if packet['class_id'] not in VisionConfig.CLASS_NAMES:
            return False

        if not 0.0 <= packet['score'] <= 1.0:
            return False

        for key in ('cx', 'cy', 'w', 'h', 'bottom_y'):
            if not 0.0 <= packet[key] <= 1.0:
                return False

        if packet['w'] <= 0.0 or packet['h'] <= 0.0:
            return False

        return True

    @staticmethod
    def _size_ratio(a, b, key):
        av = a[key]
        bv = b[key]

        if av <= 0.0001 or bv <= 0.0001:
            return 0.0

        if av < bv:
            return av / bv
        return bv / av

    def _same_target(self, incoming, reference):
        if not reference.get('valid', False):
            return False

        if incoming['class_id'] != reference['class_id']:
            return False

        if abs(incoming['cx'] - reference['cx']) > VisionConfig.RX_MATCH_CX:
            return False

        if (
            abs(incoming['bottom_y'] - reference['bottom_y'])
            > VisionConfig.RX_MATCH_BOTTOM_Y
        ):
            return False

        if (
            self._size_ratio(incoming, reference, 'w')
            < VisionConfig.RX_MIN_SIZE_RATIO
        ):
            return False

        if (
            self._size_ratio(incoming, reference, 'h')
            < VisionConfig.RX_MIN_SIZE_RATIO
        ):
            return False

        return True

    def _same_candidate(self, a, b):
        if a is None or b is None:
            return False

        if a['class_id'] != b['class_id']:
            return False

        if abs(a['cx'] - b['cx']) > VisionConfig.RX_MATCH_CX:
            return False

        if (
            abs(a['bottom_y'] - b['bottom_y'])
            > VisionConfig.RX_MATCH_BOTTOM_Y
        ):
            return False

        return True

    def _smooth_accept(self, incoming):
        """接受同一目标，并对坐标做轻量 EMA 平滑。"""
        alpha = VisionConfig.RX_SMOOTH_ALPHA

        if not self.target['valid']:
            self.target = incoming
        else:
            old = self.target
            merged = dict(incoming)

            for key in (
                'score', 'cx', 'cy', 'w', 'h', 'bottom_y'
            ):
                merged[key] = (
                    old[key] * (1.0 - alpha)
                    + incoming[key] * alpha
                )

            merged['valid'] = True
            merged['class_name'] = VisionConfig.CLASS_NAMES.get(
                merged['class_id'],
                'unknown'
            )
            self.target = merged

        self.last_good_rx_ms = time.ticks_ms()
        self.missing_since_ms = None
        self.candidate = None
        self.candidate_count = 0
        self.stable_count += 1

    def _accept_new_target(self, incoming):
        self.target = dict(incoming)
        self.target['valid'] = True
        self.target['class_name'] = VisionConfig.CLASS_NAMES.get(
            self.target['class_id'],
            'unknown'
        )

        self.last_good_rx_ms = time.ticks_ms()
        self.missing_since_ms = None
        self.candidate = None
        self.candidate_count = 0
        self.stable_count = 1

    def _handle_target_packet(self, incoming):
        if not self._valid_packet_values(incoming):
            return

        # 已锁定目标：同一目标立即平滑更新。
        if self.target['valid'] and self._same_target(incoming, self.target):
            self._smooth_accept(incoming)
            return

        # 未锁定时，要求连续若干帧确认。
        confirm_needed = VisionConfig.RX_CONFIRM_FRAMES

        # 已锁定但突然跳到别的目标时，使用更严格的切换帧数。
        if self.target['valid']:
            confirm_needed = VisionConfig.RX_SWITCH_CONFIRM_FRAMES

        if self._same_candidate(incoming, self.candidate):
            self.candidate_count += 1
        else:
            self.candidate = dict(incoming)
            self.candidate_count = 1

        # 候选尚未确认时继续保留旧目标，不让控制量突变。
        if self.candidate_count < confirm_needed:
            return

        self._accept_new_target(incoming)

    def _handle_no_target(self):
        """
        短暂 N 不立刻清空目标。
        只有连续超过 N_HOLD_MS 后才真正判无目标。
        """
        now = time.ticks_ms()

        if self.missing_since_ms is None:
            self.missing_since_ms = now
            return

        if (
            time.ticks_diff(now, self.missing_since_ms)
            >= VisionConfig.N_HOLD_MS
        ):
            self.reset_target()

    def parse_uart_line(self, line):
        self.stop_signal = 0
        line = line.strip()
        self.last_vision_line = line

        # 只打印到主板终端，不改变通信协议。
        print(line)

        if not line:
            return

        if line == 'YN':
            self._handle_yaw_line_none()
            return

        if line.startswith('Y,'):
            parts = line.split(',')

            if len(parts) != 4:
                return

            try:
                angle_i = int(parts[1])
                quality = int(parts[2])
                cy_i = int(parts[3])
            except Exception:
                return

            self._handle_yaw_line_packet(
                angle_i / 100.0,
                quality,
                self._clamp01(
                    cy_i / 1000.0
                )
            )
            return

        if line == 'N':
            self._handle_no_target()
            return

        if line == 'STOP':
            self.stop_signal = 1
            return

        # 未知文本不再被误判成 STOP；直接忽略。
        if not line.startswith('T,'):
            return

        parts = line.split(',')
        if len(parts) != 8:
            return

        try:
            class_id = int(parts[1])
            score_i = int(parts[2])
            cx_i = int(parts[3])
            cy_i = int(parts[4])
            w_i = int(parts[5])
            h_i = int(parts[6])
            by_i = int(parts[7])
        except Exception:
            return

        incoming = {
            'valid': True,
            'class_id': class_id,
            'class_name': VisionConfig.CLASS_NAMES.get(
                class_id,
                'unknown'
            ),
            'score': score_i / 1000.0,
            'cx': self._clamp01(cx_i / 1000.0),
            'cy': self._clamp01(cy_i / 1000.0),
            'w': self._clamp01(w_i / 1000.0),
            'h': self._clamp01(h_i / 1000.0),
            'bottom_y': self._clamp01(by_i / 1000.0),
        }

        self._handle_target_packet(incoming)

    def poll_uart(self):
        # 一次把当前缓存里的多行都读掉，避免旧数据积压。
        while self.uart.any():
            try:
                line = self.uart.readline()

                if not line:
                    break

                if isinstance(line, bytes):
                    line = line.decode().strip()

                self.parse_uart_line(line)

            except Exception as error:
                print('VISION UART ERROR:', error)
                break

    def target_timeout(self):
        if not self.target['valid']:
            return True

        dt = time.ticks_diff(
            time.ticks_ms(),
            self.last_good_rx_ms
        )

        return dt > VisionConfig.TARGET_TIMEOUT_MS

    def is_valid(self):
        return self.target['valid'] and not self.target_timeout()

    def is_stable(self):
        return (
            self.is_valid()
            and self.stable_count >= VisionConfig.SEARCH_STABLE_FRAMES
        )

    def get_error_x(self):
        if not self.is_valid():
            return 0.0

        return (
            self.target['cx']
            - 0.5
            - VisionConfig.CENTER_OFFSET_X
        )

    def get_distance_hint(self):
        if not self.is_valid():
            return 0.0

        return self.target['bottom_y']

    def get_class_id(self):
        if not self.is_valid():
            return -1

        return self.target['class_id']

    def get_realdist_w(self, index=None):
        if not self.is_valid():
            return 0.0

        if index is None:
            index = self.get_class_id()

        idr = [10, 10, 8.75, 9, 9]

        if not 0 <= index < len(idr):
            return 0.0

        width = self.target['w']
        if width <= 0.0001:
            return 0.0

        return self.get_error_x() / width * idr[index]

    def send_switch_command(self, class_id):
        """保留原有 C,n 类别命令。"""
        message = 'C,{}\n'.format(class_id)
        self.uart.write(message)

    def send_lower_roi_command(self, enabled):
        """
        enabled=True：摄像头只接受画面下部候选，命令 R,1。
        enabled=False：恢复全画面候选，命令 R,0。

        摄像头不返回 ACK，因此这里重复发送两次提高可靠性。
        """
        if enabled:
            message = VisionConfig.LOWER_ROI_ON_COMMAND
        else:
            message = VisionConfig.LOWER_ROI_OFF_COMMAND

        repeat = VisionConfig.ROI_COMMAND_REPEAT

        for index in range(repeat):
            self.uart.write(message)

            if index + 1 < repeat:
                time.sleep_ms(
                    VisionConfig.ROI_COMMAND_GAP_MS
                )

        print('>>> CAMERA LOWER ROI:', enabled)

    def _send_camera_control_command(self, message):
        """
        摄像头控制命令不返回 ACK，因此重复发送以提高可靠性。

        优先复用当前 VisionConfig 中已有的 ROI 重发参数；
        没有这些字段时使用安全默认值。
        """
        repeat = getattr(
            VisionConfig,
            'ROI_COMMAND_REPEAT',
            2
        )
        gap_ms = getattr(
            VisionConfig,
            'ROI_COMMAND_GAP_MS',
            20
        )

        for index in range(repeat):
            self.uart.write(message)

            if index + 1 < repeat:
                time.sleep_ms(gap_ms)

    def send_recognition_command(self, enabled):
        """
        enabled=False：
            发送 V,0，摄像头停止黄线和模型识别，并停止发送 N/T/STOP。

        enabled=True：
            发送 V,1，摄像头恢复识别。

        关闭时主板立即清空本地目标，防止继续使用旧坐标。
        """
        enabled = bool(enabled)

        if enabled:
            message = 'V,1\n'
        else:
            message = 'V,0\n'

        self._send_camera_control_command(
            message
        )

        self.recognition_enabled = enabled

        if not enabled:
            self.reset_target()
            self.stop_signal = 0

        print(
            '>>> CAMERA RECOGNITION:',
            enabled
        )

    def send_red_sandbag_enabled(self, enabled):
        """
        enabled=False：
            发送 RED,0，摄像头忽略模型类别 0，防止红砖被当成红沙包。

        enabled=True：
            发送 RED,1，恢复红沙包识别。

        关闭红沙包时，若主板当前保存的是类别 0，则立即清空。
        """
        enabled = bool(enabled)

        if enabled:
            message = 'RED,1\n'
        else:
            message = 'RED,0\n'

        self._send_camera_control_command(
            message
        )

        self.red_sandbag_enabled = enabled

        if (
            not enabled
            and self.target.get(
                'class_id',
                -1
            ) == 0
        ):
            self.reset_target()

        print(
            '>>> CAMERA RED SANDBAG:',
            enabled
        )


    def send_yaw_line_mode(self, enabled):
        """
        enabled=True：
            发送 IMU,1，摄像头只执行黄线航向测量，返回 Y/YN。

        enabled=False：
            发送 IMU,0，摄像头恢复普通 N/T/STOP 流程。

        摄像头不返回 ACK，因此重复发送。
        """
        enabled = bool(enabled)

        if enabled:
            message = VisionConfig.YAW_LINE_ON_COMMAND
        else:
            message = VisionConfig.YAW_LINE_OFF_COMMAND

        self._send_camera_control_command(
            message
        )

        self.yaw_line_mode = enabled

        if enabled:
            self.reset_target()
            self.stop_signal = 0
            self.reset_yaw_line()

        print(
            '>>> CAMERA YAW LINE MODE:',
            enabled
        )

