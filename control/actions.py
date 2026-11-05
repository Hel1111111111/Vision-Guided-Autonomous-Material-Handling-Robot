# actions.py
# 动作层：把旧版 main.py 中散落的 wait/go_straight/go_h/yaw_correct/h_correct 等逻辑封装成可复用函数。

import time
from config import (
    MotionConfig,
    TaskConfig,
    CoopConfig,
    GuideConfig,
    YellowYawConfig
)

class ActionController:
    def __init__(self, system):
        self.sys = system

        self.motor = system.motor
        self.vision = system.vision
        self.imu = system.imu
        self.wireless = system.wireless

    def set_state(self, state):
        self.sys.state_line = state
        
    @staticmethod
    def _wrap_angle(angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    @staticmethod
    def _clamp(value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def wait(self, duration_ms):
        self.set_state("wait")
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            self.motor.set_velocity(0, 0, 0)
            time.sleep_ms(1)

    def timed_move(self, duration_ms, vx, vy, w):
        """
        w 参数保留仅为兼容旧调用签名，实际不再使用：航向由进入时的 IMU yaw
        闭环保持，见 GuideConfig.YAW_KP / MAX_YAW_RATE / YAW_HOLD_DEADZONE_DEG。
        """
        target_yaw = self.imu.get_yaw()
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            err_yaw = self._wrap_angle(target_yaw - self.imu.get_yaw())
            if abs(err_yaw) < GuideConfig.YAW_HOLD_DEADZONE_DEG+2:
                w_cmd = 0.0
            else:
                w_cmd = self._clamp(-2.5*GuideConfig.YAW_KP * err_yaw, GuideConfig.MAX_YAW_RATE)
            self.motor.set_velocity(vx, vy, w_cmd)
            time.sleep_ms(1)

    def go_straight(self, duration_ms, speed=MotionConfig.SPEED_START_FORWARD):
        self.set_state("go_straight")
        self.timed_move(duration_ms, speed, 0, 0)

    def go_lateral(self, duration_ms, direction):
        self.set_state("go_lateral")
        if direction == -1:
            self.timed_move(duration_ms, 0, -MotionConfig.SPEED_START_LATERAL, 0)
        elif direction == 1:
            self.timed_move(duration_ms, 0, MotionConfig.SPEED_START_LATERAL, 0)
        else:
            self.wait(duration_ms)
    
    def get_wr(self):       
        if int(self.vision.get_distance_hint()*1170.64-265.45)>0:
            return int(self.vision.get_distance_hint()*1170.64-265.45)+40 #80
        else:
            return 500

    def yaw_correct(self, mode, timeout_ms=12000):
        w=self.get_wr()
        self.set_state("yaw_correct")
        """
        旧版 yaw_correct(dre) 的结构化版本。
        mode=0: 转到 +90° 附近
        mode=1: 转到 180° 附近
        mode=2: 转到 -90° 附近
        mode=3: 转回 0° 附近
        """
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            yaw = self.imu.get_yaw()

            if mode == 3 or mode == 4:
                if yaw > 90 or yaw < -90:
                    self.motor.set_velocity(0, -1200, w)
                else:
                    self.motor.set_velocity(0, 1200, -w)
                if yaw > 85 and yaw < 95:
                    break

            elif mode == 2:
                self.motor.set_velocity(0, -1200, w)
                if abs(yaw) > 175:
                    break

            elif mode == 0 or mode == 1:
                if yaw > 90 or yaw < -90:
                    self.motor.set_velocity(0, 1200, -w)
                else:
                    self.motor.set_velocity(0, -1200, w)
                if yaw < -85 and yaw > -95:
                    break

            elif mode == 5:
                if yaw > 0:
                    self.motor.set_velocity(0, -1200, w)
                else:
                    self.motor.set_velocity(0, 1200, -w)
                if abs(yaw) < 7:
                    break

            time.sleep_ms(1)
        self.motor.stop()
        
    def yaw_c_self(self, timeout_ms=12000):
        """
        推物完成、后退之后，主车仅发送 w 原地自转回到场地航向。

        该阶段临时广播 weight_mode=4，使车2使用第二套自转切向补偿：
            SPIN_COMP_RETURN_GAIN_VX = -1.5
            SPIN_COMP_RETURN_GAIN_VY = -0.4

        其他转向函数（例如识别到物体后的 yaw_correct）仍使用普通模式，
        因而继续采用原来的 -1.2 / -0.3。
        """
        self.set_state("yaw_c_self_return_spin")

        old_mode = self.sys.velocity_weight_mode
        self.sys.velocity_weight_mode = CoopConfig.RETURN_SPIN_WEIGHT_MODE

        # 等待约 3 个无线周期，让车2先收到 mode=4。
        time.sleep_ms(CoopConfig.WEIGHT_MODE_SETTLE_MS)

        start = time.ticks_ms()
        try:
            while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
                yaw = self.imu.get_yaw()

                # 这一过程只有 w，正是第二套补偿参数的使用阶段。
                self.motor.set_velocity(0, 0, 1200)

                if abs(yaw) < 7:
                    break

                time.sleep_ms(1)
        finally:
            self.motor.stop()

            # 先保持 mode=4 发送若干帧零速度，避免车2在最后一帧仍保留转动。
            time.sleep_ms(CoopConfig.WEIGHT_MODE_SETTLE_MS)

            # 恢复进入函数前的模式，通常为 0。
            self.sys.velocity_weight_mode = old_mode

    def search_until_visible(self, count, direction_speed, threshold=0.001, timeout_ms=None):
        HARD_TIMEOUT_MS = 30000
        OUT_WAIT_MS = 300
        OUT_FORWARD_MS = 600
        OUT_FORWARD_SPEED = -1500
        SAFE_MARGIN = 400
        YCOUNT=[-1700,-1900,-2800]
        Y_MIN=YCOUNT[count]
        Y_MAX =-490

        effective_timeout = HARD_TIMEOUT_MS
        if timeout_ms is not None and timeout_ms < HARD_TIMEOUT_MS:
            effective_timeout = timeout_ms

        # 进入时锁定航向，搜索/出界纠正全程都靠这个 target_yaw 闭环保持，
        # 增益沿用 timed_move 里已经调好的 3*YAW_KP / MAX_YAW_RATE。
        target_yaw = self.imu.get_yaw()

        def hold_w():
            err_yaw = self._wrap_angle(target_yaw - self.imu.get_yaw())
            if abs(err_yaw) < GuideConfig.YAW_HOLD_DEADZONE_DEG+2:
                return 0.0
            return self._clamp(-3 * GuideConfig.YAW_KP * err_yaw, GuideConfig.MAX_YAW_RATE)

        start = time.ticks_ms()
        cur_speed = direction_speed

        while abs(self.vision.get_error_x()) < threshold or (not self.vision.is_valid()):
            if time.ticks_diff(time.ticks_ms(), start) > effective_timeout:
                self.motor.set_velocity(0, 0, 0)
                return False

            x, y = self.sys.odo.get_position()
            if y < Y_MIN or y > Y_MAX:
                self.motor.set_velocity(0, 0, 0)
                self.wait(OUT_WAIT_MS)
                self.timed_move(OUT_FORWARD_MS, OUT_FORWARD_SPEED, 0, 0)
                cur_speed = -cur_speed

                # 以反向速度持续搜索，直到确实回到安全范围内、
                # 或中途看见目标、或总时长超时，避免每 1ms 就重判一次出界。
                while True:
                    if time.ticks_diff(time.ticks_ms(), start) > effective_timeout:
                        self.motor.set_velocity(0, 0, 0)
                        return False
                    if not (abs(self.vision.get_error_x()) < threshold or (not self.vision.is_valid())):
                        break
                    x, y = self.sys.odo.get_position()
                    if (Y_MIN + SAFE_MARGIN) <= y <= (Y_MAX - SAFE_MARGIN):
                        break
                    self.motor.set_velocity(0, cur_speed, hold_w())
                    time.sleep_ms(1)
                continue

            self.motor.set_velocity(0, cur_speed, hold_w())
            time.sleep_ms(1)

        self.motor.set_velocity(0, 0, 0)
        return True

    def align_fixed_direction(self, direction_speed, threshold, timeout_ms=None):
        start = time.ticks_ms()
        while abs(self.vision.get_error_x()) > threshold:
            self.motor.set_velocity(0, direction_speed, 0)
            if timeout_ms is not None and time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                return False
            time.sleep_ms(1)
        self.motor.stop()
        return True

    def horizontal_correct(self, class_id):
        self.state_line="horizontal_correct"
        """
        旧版 h_correct(idd) 的封装版本。
        包含：搜索目标、根据误差变化估计剩余移动时间、最后反向刹车 100ms。
        """
        cfg = TaskConfig.H_CORRECT_CFG.get(class_id, TaskConfig.H_CORRECT_CFG[0])
        search_th = cfg['SEARCH_TH']
        predict_k = cfg['PREDICT_K']
        err_over = cfg['ERR_OVER']

        move_speed = MotionConfig.SPEED_H_CORRECT
        flag_rl = 1

        # 第一阶段：没看到目标时左右来回搜索
        search_ts = time.ticks_ms()
        while abs(self.vision.get_error_x()) < 0.001:
            elapsed = time.ticks_diff(time.ticks_ms(), search_ts)
            if elapsed < 1500:
                self.motor.set_velocity(0, 1200, 0)
            elif elapsed < 3000:
                self.motor.set_velocity(0, -1200, 0)
            else:
                search_ts = time.ticks_ms()
            time.sleep_ms(1)

        all_err_x = self.vision.get_error_x()
        max_time = int(abs(all_err_x) * 150000)
        if max_time < 1500:
            max_time = 1500

        global_ts = time.ticks_ms()
        last_err_x = self.vision.get_error_x()
        now_err_x = self.vision.get_error_x()
        count_delta_t = 1
        k_now = 0
        t_move = 4000

        while abs(self.vision.get_error_x()) > search_th:
            if time.ticks_diff(time.ticks_ms(), global_ts) > max_time:
                break
            if not self.vision.is_valid():
                break

            last_err_x = now_err_x
            now_err_x = self.vision.get_error_x()
            delta_err_x = now_err_x - last_err_x

            if abs(delta_err_x) < 0.001:
                count_delta_t += 1
            else:
                k = delta_err_x / count_delta_t
                count_delta_t = 1
                if abs(k_now) > 0.0001:
                    k_now = (k_now + k) / 2.0
                else:
                    k_now = k

                if abs(k_now) > 0.0001:
                    t_move = abs((now_err_x / k_now) * predict_k)
                    if t_move > max_time:
                        t_move = max_time

            dead_zone = 0.02
            if abs(now_err_x) < dead_zone:
                self.motor.set_velocity(0, 0, 0)
                break

            if now_err_x > 0:
                self.motor.set_velocity(0, -move_speed, 0)
                flag_rl = 1
            else:
                self.motor.set_velocity(0, move_speed, 0)
                flag_rl = -1

            if time.ticks_diff(time.ticks_ms(), global_ts) > t_move:
                if abs(self.vision.get_error_x()) > err_over:
                    continue
                break
            time.sleep_ms(1)

        # 旧版最后 100ms 反向刹车
        stop_ts = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), stop_ts) < 100:
            self.motor.set_velocity(0, -flag_rl * move_speed, 0)
            time.sleep_ms(1)
        self.motor.stop()

    def approach_until_distance(self, target_dist, speed=MotionConfig.SPEED_APPROACH, timeout_ms=None):
        start = time.ticks_ms()
        while self.vision.get_distance_hint() < target_dist:
            self.motor.set_velocity(speed, 0, 0)
            if timeout_ms is not None and time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                return False
            time.sleep_ms(1)
        self.motor.stop()
        return True

    def turn_delivery_by_id(self, class_id, timeout_ms=12000):
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            yaw = self.imu.get_yaw()
            if class_id == 0 or class_id == 1:
                if yaw > 90 or yaw < -90:
                    self.motor.set_velocity(0, 1500, -800)
                else:
                    self.motor.set_velocity(0, -1500, 800)
                if yaw < -85 and yaw > -95:
                    break
            elif class_id == 2:
                if yaw > 0:
                    self.motor.set_velocity(0, -1500, 800)
                else:
                    self.motor.set_velocity(0, 1500, -800)
                if abs(yaw) < 5:
                    break
            elif class_id == 3 or class_id == 4:
                if yaw > 90 or yaw < -90:
                    self.motor.set_velocity(0, -1500, 800)
                else:
                    self.motor.set_velocity(0, 1500, -800)
                if yaw > 85 and yaw < 95:
                    break
            time.sleep_ms(1)
        self.motor.stop()

    def yaw_recover_after_delivery(self, class_id):
        if class_id == 0 or class_id == 1:
            self.yaw_correct(2)
        elif class_id == 2:
            self.yaw_correct(3)
        elif class_id == 3 or class_id == 4:
            self.yaw_correct(0)


    def correct_imu_by_yellow_line(
        self,
        expected_yaw_deg,
        timeout_ms=None
    ):
        """
        车辆已经完成推出动作并停车后，利用画面中的黄线校正 IMU。

        expected_yaw_deg：
            当前车身机械上对正该边界时应对应的绝对 yaw，
            由 YellowYawConfig.EXPECTED_YAW_BY_CLASS 提供。

        返回：
            True：收到稳定黄线并成功融合；
            False：超时或安全检查拒绝。
        """
        self.set_state(
            "yellow_line_imu_correction"
        )

        self.motor.stop()
        time.sleep_ms(
            YellowYawConfig.VEHICLE_SETTLE_MS
        )

        if timeout_ms is None:
            timeout_ms = (
                YellowYawConfig.ACQUIRE_TIMEOUT_MS
            )

        old_dynamic_bias = (
            self.imu.enable_dynamic_bias
        )

        # 校正期间禁止动态零偏更新，避免停车状态下两种校正同时发生。
        self.imu.set_dynamic_bias_enable(False)

        self.vision.send_yaw_line_mode(True)

        start = time.ticks_ms()
        success = False

        try:
            while True:
                self.motor.set_velocity(
                    0,
                    0,
                    0
                )

                if self.vision.is_yaw_line_stable():
                    measured_line_angle = (
                        self.vision
                        .get_yaw_line_angle()
                    )

                    if measured_line_angle is None:
                        time.sleep_ms(1)
                        continue

                    line_error = self._wrap_angle(
                        measured_line_angle
                        - YellowYawConfig
                        .CAMERA_LINE_ZERO_DEG
                    )

                    # 黄线是直线，方向以 180° 为周期。
                    while line_error > 90.0:
                        line_error -= 180.0

                    while line_error < -90.0:
                        line_error += 180.0

                    visual_absolute_yaw = (
                        float(expected_yaw_deg)
                        + YellowYawConfig
                        .LINE_TO_YAW_SIGN
                        * YellowYawConfig
                        .LINE_TO_YAW_SCALE
                        * line_error
                    )

                    visual_absolute_yaw = (
                        self._wrap_angle(
                            visual_absolute_yaw
                        )
                    )

                    old_yaw = self.imu.get_yaw()

                    ok, applied = (
                        self.imu
                        .apply_external_yaw_reference(
                            visual_absolute_yaw,
                            gain=(
                                YellowYawConfig
                                .CORRECTION_GAIN
                            ),
                            max_step_deg=(
                                YellowYawConfig
                                .MAX_CORRECTION_STEP_DEG
                            ),
                            hard_reject_deg=(
                                YellowYawConfig
                                .HARD_REJECT_ERROR_DEG
                            ),
                            min_correction_deg=(
                                YellowYawConfig
                                .MIN_CORRECTION_DEG
                            )
                        )
                    )

                    print(
                        "COUNT3 YAW CORRECTION",
                        "line:",
                        round(
                            measured_line_angle,
                            2
                        ),
                        "line_error:",
                        round(line_error, 2),
                        "expected:",
                        round(
                            expected_yaw_deg,
                            2
                        ),
                        "visual_yaw:",
                        round(
                            visual_absolute_yaw,
                            2
                        ),
                        "imu_old:",
                        round(old_yaw, 2),
                        "applied:",
                        round(applied, 2),
                        "imu_new:",
                        round(
                            self.imu.get_yaw(),
                            2
                        ),
                        "quality:",
                        self.vision
                        .get_yaw_line_quality()
                    )

                    success = ok
                    break

                if (
                    time.ticks_diff(
                        time.ticks_ms(),
                        start
                    )
                    >= timeout_ms
                ):
                    print(
                        "COUNT3 YAW CORRECTION TIMEOUT"
                    )
                    break

                time.sleep_ms(1)

        finally:
            self.motor.stop()

            self.vision.send_yaw_line_mode(
                False
            )

            self.imu.set_dynamic_bias_enable(
                old_dynamic_bias
            )

            time.sleep_ms(
                YellowYawConfig
                .MODE_EXIT_SETTLE_MS
            )

        return success

    def push_until_stop_signal(self, task_index,flag=0):
        self.state_line="push"
        #self.sys.velocity_weight_mode = 0
        cm = 1
        make_time = TaskConfig.MAKE_TIME_BY_INDEX[task_index]
        start = time.ticks_ms()

        start_low_speed_ms = min(500, make_time)

        self.vision.stop_signal = 0
        while self.vision.stop_signal == 0:
            elapsed = time.ticks_diff(time.ticks_ms(), start)
            if elapsed < start_low_speed_ms:
                self.motor.set_velocity(-1000, 0, 0)
            elif elapsed < make_time:
                self.motor.set_velocity(MotionConfig.SPEED_PUSH, 0, 0)
            else:
                if time.ticks_diff(time.ticks_ms(), start) > 8000:
                    cm = 0
                    self.sys.velocity_weight_mode = 0
                    break
                else:
                    self.vision.stop_signal = 0
            time.sleep_ms(1)

        if cm == 1:
            self.timed_move(600, MotionConfig.SPEED_PUSH, 0, 0)
        else:
            self.timed_move(600, MotionConfig.SPEED_BACK, 0, 0)
        self.motor.stop()
        #self.sys.velocity_weight_mode = 0
        return cm
    
    def return_to_dock(self, guide, forward_speed=-2500, lateral_speed=2500,
                        min_lateral_ms=1000, x_threshold=400, goto_timeout_ms=GuideConfig.DEFAULT_TIMEOUT_MS):
        """
        回车程序：
        1. 用 guide.goto_position 惯导回到 (-500, -500)（快速模式）。
        2. yaw_correct(0) 把航向纠正到 -90° 附近。
        3. 以 forward_speed 前进，航向闭环保持在 -90°，直到收到摄像头 STOP 停止命令
           （self.vision.stop_signal 置 1，与 push_until_stop_signal 用的是同一个标志）。
        4. 切换为横向移动（lateral_speed），航向仍闭环保持 -90°，
           直到"至少移动 min_lateral_ms 毫秒"且"里程计 x 坐标 < x_threshold"同时满足才停车。
        """
        self.set_state("return_to_dock")

        guide.goto_position(-300, -600, hold_yaw=0.0, precise=False,
                             min_creep_speed=2500, timeout_ms=goto_timeout_ms)
        self.wait(500)
        self.yaw_correct(0)
        target_yaw = -90.0

        def hold_w():
            err_yaw = self._wrap_angle(target_yaw - self.imu.get_yaw())
            if abs(err_yaw) < GuideConfig.YAW_HOLD_DEADZONE_DEG+2:
                return 0.0
            return self._clamp(-GuideConfig.YAW_KP * err_yaw, GuideConfig.MAX_YAW_RATE)

        self.set_state("return_to_dock_forward")
        self.vision.stop_signal = 0
        while self.vision.stop_signal == 0:
            x, y = self.sys.odo.get_position()
            if y>1000:
                guide.goto_position(-100, -100, hold_yaw=0.0, precise=False,
                               min_creep_speed=2500, timeout_ms=5000)
            
            # 再走到 (100, 100)
                guide.goto_position(100, 100, hold_yaw=0.0, precise=False,
                               min_creep_speed=2500, timeout_ms=5000)
                return
            self.motor.set_velocity(forward_speed, 0, hold_w())
            time.sleep_ms(1)

        self.set_state("return_to_dock_lateral")
        lateral_start = time.ticks_ms()
        while True:
            elapsed = time.ticks_diff(time.ticks_ms(), lateral_start)
            x, y = self.sys.odo.get_position()
            if elapsed >= min_lateral_ms and x > x_threshold:
                break
            self.motor.set_velocity(0, lateral_speed, hold_w())
            time.sleep_ms(1)


    def lateral_to_center(self):
        RATE_ERRtoT=75
        cnt_0=0
        err_t0=0
        while self.sys.realerr_c==0:
            time.sleep_ms(2)
            cnt_0+=1
            if cnt_0==2:
                break
        err_t0=abs(2-self.sys.realerr_c)*RATE_ERRtoT
        if (5-self.sys.realerr_c)>=0:
            sig=1
        else:
            sig=-1
        self.go_lateral(err_t0,sig)
        
    def lateral_to_ready(self,index):
        RATE_ERRtoT=80
        IDR=[10,9,6.0,7,7] #6.8网球
        cnt_0=0
        err_t0=0
        while self.sys.realerr_c==0:
            time.sleep_ms(2)
            cnt_0+=1
            if cnt_0==15:
                break
        err_t0=abs(IDR[index]-self.sys.realerr_c)*RATE_ERRtoT
        if (IDR[index]-self.sys.realerr_c)>=0:
            sig=1
        else:
            sig=-1
        self.go_lateral(err_t0,sig)

    def search_until_visible1(self, direction_speed, threshold=0.001, timeout_ms=None):
        self.set_state("search_until_visible")
        start = time.ticks_ms()
        while abs(self.vision.get_error_x()) < threshold or (not self.vision.is_valid()):
            self.motor.set_velocity(0, direction_speed, 0)
            if timeout_ms is not None and time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                return False
            time.sleep_ms(1)
        return True  





