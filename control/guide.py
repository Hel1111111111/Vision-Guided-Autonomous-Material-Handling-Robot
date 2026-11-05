# guide.py
# 惯导导航层：基于 odometry.py 给出的全局坐标，闭环控制小车走到场地内指定的 (x, y)。
#guide 不是固定速度，是比例控制器，速度随距离自动变化。见vx_cmd，vy_cmd

############################################
#y正：左
#x正：后
#主要接口：ok标识 = guide.goto_position(X距离, Y距离, hold_yaw=相较于出发点转动角度,
#                                      precise=False)
#precise模式：默认模式，接近目标点比例减速+死区补偿，更精确，速度偏慢。
#fast模式（precise=False）：方向锁定+滞回，目标方向角变化超过
#   GuideConfig.FAST_LOCK_ANGLE_DEG 才重新锁定方向，否则沿用上次锁定方向，
#   全程按 GuideConfig.FAST_LOCK_SPEED 固定速度冲向目标，不减速、不做逐轴死区
#   补偿，到点(FAST_POS_TOLERANCE_MM)直接硬停。可以过冲，但避免了接近目标点时
#   因位置噪声反复调整方向走Z字形的问题。min_creep_speed 参数在此模式下不生效。
###########################################

import math
import time
from config import GuideConfig


class GuideController:
    def __init__(self, system, odometry):
        self.sys = system
        self.motor = system.motor
        self.odo = odometry

    def _wrap_angle(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _clamp(self, value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def goto_position(self, target_x, target_y, hold_yaw=0.0,
                       precise=True, tolerance_mm=None, slowdown_radius_mm=None,
                       min_creep_speed=None,
                       timeout_ms=GuideConfig.DEFAULT_TIMEOUT_MS,
                       debug=False, wireless_debug=False, ff_only_ms=None):
        """
        闭环导航到全局坐标 (target_x, target_y)，单位 mm，参考系是最近一次
        odometry.reset() 时的位置和航向。

        hold_yaw: 导航过程中保持的航向角（度），一般填出发时的航向（通常是 0）。

        precise: True=精确模式（默认），接近目标点时比例减速+死区补偿，到点判定严，
                 适合需要精确停位的任务；
                 False=快速模式，方向锁定+滞回后固定速度冲向目标，不减速，到点判定松，
                 可以过冲，适合只要求"最快跑到大致位置"的任务。
        tolerance_mm: 到点判定容差(mm)，不传时按 precise 取默认值
                 (精确=GuideConfig.POS_TOLERANCE_MM，快速=GuideConfig.FAST_POS_TOLERANCE_MM)，
                 也可以直接传值给某个任务单独设置定位精度。
        slowdown_radius_mm: 减速区半径(mm)，仅精确模式使用，不传时自动取
                 max(GuideConfig.SLOWDOWN_RADIUS_MM, tolerance_mm * SLOWDOWN_RADIUS_RATIO)。
                 快速模式不减速，这个参数会被忽略。
        min_creep_speed: 死区补偿的最低速度，仅精确模式使用，不传时用 GuideConfig.MIN_CREEP_SPEED。
                 快速模式改用方向锁定（GuideConfig.FAST_LOCK_ANGLE_DEG/FAST_LOCK_SPEED），
                 这个参数在快速模式下不生效。
        ff_only_ms: 不传/None 时不生效（不改 sys.velocity_weight_mode，行为和以前一样）。
                 传入毫秒数时，本次导航开始的前 ff_only_ms 毫秒把 sys.velocity_weight_mode
                 置为 GuideConfig.FF_ONLY_WEIGHT_MODE（车2只用前馈，不看红外纠偏），
                 之后切到 GuideConfig.COMBINED_WEIGHT_MODE（前馈+红外融合）；函数返回前
                 统一收回到 0，避免状态残留影响下一次调用。

        到达 tolerance_mm 范围内返回 True；超时未到达则停车并返回 False。
        debug=True 时每隔约 150ms 走本地 print() 打印一次位置/误差，仅用于调参。
        wireless_debug=True 时同一节奏改走无线串口发送，脱离USB线在场地测试时用；
        无线通道同时也在跑车-车协同帧，别和 debug 同时长期开着，够用就关掉。
        """
        #在保持车头方向不变的同时，直接沿最短路径（世界坐标误差向量方向)滑动到目标点
        if tolerance_mm is None:
            tolerance_mm = GuideConfig.POS_TOLERANCE_MM if precise else GuideConfig.FAST_POS_TOLERANCE_MM
        if precise and slowdown_radius_mm is None:
            slowdown_radius_mm = max(GuideConfig.SLOWDOWN_RADIUS_MM,
                                      tolerance_mm * GuideConfig.SLOWDOWN_RADIUS_RATIO)
        if min_creep_speed is None:
            min_creep_speed = GuideConfig.MIN_CREEP_SPEED

        start = time.ticks_ms()
        last_print = start
        locked_angle = None  # 仅快速模式使用：锁定的目标方向角（弧度，世界坐标）

        while True:
            # 调参阶段的手动急停：拨动 SWITCH2 立即停车，防止增益不对时车冲出场地/撞人。
            self.sys.velocity_weight_mode = 3
            if self.sys.should_stop_by_switch():
                self.motor.stop()
                if ff_only_ms is not None:
                    self.sys.velocity_weight_mode = 0
                return False

            if ff_only_ms is not None:
                if time.ticks_diff(time.ticks_ms(), start) < ff_only_ms:
                    self.sys.velocity_weight_mode = GuideConfig.FF_ONLY_WEIGHT_MODE
                else:
                    self.sys.velocity_weight_mode = GuideConfig.COMBINED_WEIGHT_MODE

            x, y = self.odo.get_position()
            yaw = self.odo.get_yaw()

            err_x_world = target_x - x
            err_y_world = target_y - y
            dist = math.sqrt(err_x_world * err_x_world + err_y_world * err_y_world)

            if dist < tolerance_mm:
                break
            if timeout_ms is not None and time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                self.motor.stop()
                if ff_only_ms is not None:
                    self.sys.velocity_weight_mode = 0
                return False

            yaw_rad = math.radians(yaw)
            cos_y = math.cos(yaw_rad)
            sin_y = math.sin(yaw_rad)

            # 世界坐标误差旋转回车体坐标，因为 set_velocity 的 vx/vy 是车体坐标；也给debug打印用。
            err_x_body = err_x_world * cos_y + err_y_world * sin_y
            err_y_body = -err_x_world * sin_y + err_y_world * cos_y

            err_yaw = self._wrap_angle(hold_yaw - yaw)
            w_cmd = self._clamp(-GuideConfig.YAW_KP * err_yaw, GuideConfig.MAX_YAW_RATE)

            if precise:
                vx_cmd = GuideConfig.POS_KP * err_x_body
                vy_cmd = GuideConfig.POS_KP * err_y_body

                # 精确模式：接近目标点时按比例减速，避免冲过头。
                if dist < slowdown_radius_mm:
                    scale = dist / slowdown_radius_mm
                    if scale < GuideConfig.MIN_SPEED_SCALE:
                        scale = GuideConfig.MIN_SPEED_SCALE
                    vx_cmd *= scale
                    vy_cmd *= scale
                    w_cmd *= scale

                # 死区补偿：指令非零但幅值小于 min_creep_speed 时推不动轮子，兜到最小幅值。
                if 0.0 < vx_cmd < min_creep_speed:
                    vx_cmd = min_creep_speed
                elif -min_creep_speed < vx_cmd < 0.0:
                    vx_cmd = -min_creep_speed
                if 0.0 < vy_cmd < min_creep_speed:
                    vy_cmd = min_creep_speed
                elif -min_creep_speed < vy_cmd < 0.0:
                    vy_cmd = -min_creep_speed
            else:
                # 快速模式：方向锁定+滞回。目标方向角变化超过 FAST_LOCK_ANGLE_DEG 才重新锁定，
                # 否则沿用上一次锁定方向，全程按固定速度幅值冲向锁定方向。
                # 不做逐轴死区补偿：逐轴补偿在误差小、噪声占比大时会把误差方向扭曲成另一个
                # 方向，才是接近目标点反复调整方向走Z字形的主因。
                target_angle = math.atan2(err_y_world, err_x_world)
                if locked_angle is None:
                    locked_angle = target_angle
                else:
                    angle_diff = self._wrap_angle(math.degrees(target_angle - locked_angle))
                    if abs(angle_diff) > GuideConfig.FAST_LOCK_ANGLE_DEG:
                        locked_angle = target_angle

                dir_x_world = math.cos(locked_angle)
                dir_y_world = math.sin(locked_angle)
                vx_cmd = GuideConfig.FAST_LOCK_SPEED * (dir_x_world * cos_y + dir_y_world * sin_y)
                vy_cmd = GuideConfig.FAST_LOCK_SPEED * (-dir_x_world * sin_y + dir_y_world * cos_y)

            vx_cmd = self._clamp(vx_cmd, GuideConfig.MAX_SPEED)
            vy_cmd = self._clamp(vy_cmd, GuideConfig.MAX_SPEED)

            self.motor.set_velocity(vx_cmd, vy_cmd, w_cmd)

            if (debug or wireless_debug) and time.ticks_diff(time.ticks_ms(), last_print) > 150:
                last_print = time.ticks_ms()
                if debug:
                    print('dist={:.1f} err_body=({:.1f},{:.1f}) yaw={:.2f} err_yaw={:.2f} vx={:.1f} vy={:.1f} w={:.1f}'.format(
                        dist, err_x_body, err_y_body, yaw, err_yaw, vx_cmd, vy_cmd, w_cmd
                    ))
                if wireless_debug:
                    msg = 'x:{:.1f}\r\ny:{:.1f}\r\ndist:{:.1f}\r\nyaw:{:.2f}\r\n'.format(x, y, dist, yaw)
                    self.sys.wireless.send_debug_str(msg)

            time.sleep_ms(1)

        self.motor.stop()
        if ff_only_ms is not None:
            self.sys.velocity_weight_mode = 0
        return True


