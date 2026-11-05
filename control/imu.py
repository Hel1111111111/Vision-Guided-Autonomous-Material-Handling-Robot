# imu_improved.py
# 改进版 IMU 姿态解算模块。
# 目标：降低 yaw 漂移，保留原 IMUManager 接口：calibrate(), update(), get_yaw(), get_euler(), get_imu_object()。
# 使用方式：把本文件改名为 imu.py，替换工程里的原 imu.py。
#在原来算法基础上有改过吗？？
import math
import time
from seekfree import IMU660RX
from config import IMUConfig


class IMUManager:
    def __init__(self):
        self.imu = IMU660RX()
        self.imu_data = self.imu.get()

        # 零偏，单位 deg/s
        self.gyro_x_bias = 0.0
        self.gyro_y_bias = 0.0
        self.gyro_z_bias = 0.0

        # 低通滤波后的角速度，单位 deg/s
        self.gx_f = 0.0
        self.gy_f = 0.0
        self.gz_f = 0.0

        # 动态零偏估计
        self.still_count = 0

        # 四元数，主要用于 roll/pitch；yaw 会以 z 轴积分为主，避免 6轴 Mahony 对 yaw 修正能力不足导致混乱。
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        self.integralFBx = 0.0
        self.integralFBy = 0.0
        self.integralFBz = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.last_time_us = time.ticks_us()

        # 是否允许动态更新零偏。车辆剧烈运动/转向时不要更新。
        self.enable_dynamic_bias = True

        # 运行时调试变量
        self.last_gz_deg = 0.0
        self.last_acc_norm = 1.0
        self.is_still = False

        # 最近一次视觉绝对航向校正记录。
        self.last_external_yaw_correction = 0.0
        self.last_external_yaw_target = 0.0

    def get_imu_object(self):
        return self.imu

    # -----------------------------
    # 工具函数
    # -----------------------------
    def _get_cfg(self, name, default):
        return getattr(IMUConfig, name, default)

    def _wrap_angle(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _median3(self, a, b, c):
        if a > b:
            a, b = b, a
        if b > c:
            b, c = c, b
        if a > b:
            a, b = b, a
        return b

    def set_yaw_zero(self, yaw_value=0.0):
        """在已知车头方向时重置 yaw，比如发车前、视觉对准后、撞到定位边界后。"""
        self.yaw = yaw_value
        self.last_time_us = time.ticks_us()

    def set_dynamic_bias_enable(self, enable):
        """如果正在快速转弯，可以临时关闭动态零偏更新。"""
        self.enable_dynamic_bias = bool(enable)


    def apply_external_yaw_reference(
        self,
        absolute_yaw_deg,
        gain=1.0,
        max_step_deg=20.0,
        hard_reject_deg=45.0,
        min_correction_deg=0.0
    ):
        """
        将摄像头黄线得到的绝对航向融合到当前 IMU yaw。

        返回：
            (success, applied_correction_deg)

        保护：
            外部值与当前 IMU 相差过大时拒绝；
            单次修正限制最大步长；
            小修正量不改变 yaw；
            修改后重置积分时间和 Z 轴滤波残留。
        """
        current_yaw = self.yaw
        target_yaw = self._wrap_angle(
            float(absolute_yaw_deg)
        )
        error = self._wrap_angle(
            target_yaw - current_yaw
        )

        if abs(error) > float(hard_reject_deg):
            return False, 0.0

        correction = error * float(gain)

        max_step = abs(
            float(max_step_deg)
        )

        if correction > max_step:
            correction = max_step
        elif correction < -max_step:
            correction = -max_step

        if (
            abs(correction)
            < float(min_correction_deg)
        ):
            self.last_external_yaw_correction = 0.0
            self.last_external_yaw_target = target_yaw
            return True, 0.0

        self.yaw = self._wrap_angle(
            current_yaw + correction
        )

        self.last_external_yaw_correction = (
            correction
        )
        self.last_external_yaw_target = (
            target_yaw
        )

        # 避免下一次 update() 使用异常大的 dt，
        # 并清除停车前残留的 Z 轴滤波值。
        self.last_time_us = time.ticks_us()
        self.gz_f = 0.0
        self.last_gz_deg = 0.0

        return True, correction

    # -----------------------------
    # 标定：比旧版更稳
    # -----------------------------
    def calibrate(self):
        # 旧版 200 个样本偏少，这里默认用 600；如果 config.py 里写了则优先使用。
        samples = self._get_cfg('CALIBRATE_SAMPLES', 600)
        delay_ms = self._get_cfg('CALIBRATE_DELAY_MS', 3)

        # 前若干帧丢弃，避免刚上电/刚创建对象时数据不稳。
        discard = self._get_cfg('CALIBRATE_DISCARD', 50)
        for _ in range(discard):
            self.imu.read()
            time.sleep_ms(delay_ms)

        sum_x = 0.0
        sum_y = 0.0
        sum_z = 0.0
        valid = 0

        # 简单离群抑制：如果加速度模长离 1g 太远，说明车被碰了或数据不稳，这帧不参与。
        acc_to_g = self._get_cfg('ACC_LSB_TO_G', 1.0 / 4096.0)
        gyro_to_deg = self._get_cfg('GYRO_LSB_TO_DEG', 1.0 / 16.4)
        acc_tol = self._get_cfg('CALIBRATE_ACC_TOL', 0.18)

        while valid < samples:
            self.imu.read()
            ax = self.imu_data[0] * acc_to_g
            ay = self.imu_data[1] * acc_to_g
            az = self.imu_data[2] * acc_to_g
            acc_norm = math.sqrt(ax * ax + ay * ay + az * az)

            if abs(acc_norm - 1.0) < acc_tol:
                sum_x += self.imu_data[3] * gyro_to_deg
                sum_y += self.imu_data[4] * gyro_to_deg
                sum_z += self.imu_data[5] * gyro_to_deg
                valid += 1

            time.sleep_ms(delay_ms)

        self.gyro_x_bias = sum_x / samples
        self.gyro_y_bias = sum_y / samples
        self.gyro_z_bias = sum_z / samples

        self.gx_f = 0.0
        self.gy_f = 0.0
        self.gz_f = 0.0
        self.yaw = 0.0
        self.last_time_us = time.ticks_us()
        time.sleep_ms(200)

    # -----------------------------
    # Mahony 6DOF：主要修正 roll/pitch
    # -----------------------------
    def _mahony_update_6dof(self, gx, gy, gz, ax, ay, az, dt):
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:
            return

        ax /= norm
        ay /= norm
        az /= norm

        halfvx = self.q1 * self.q3 - self.q0 * self.q2
        halfvy = self.q0 * self.q1 + self.q2 * self.q3
        halfvz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3

        halfex = ay * halfvz - az * halfvy
        halfey = az * halfvx - ax * halfvz
        halfez = ax * halfvy - ay * halfvx

        two_kp = self._get_cfg('TWO_KP', 0.6)
        two_ki = self._get_cfg('TWO_KI', 0.0)

        if two_ki > 0.0:
            self.integralFBx += two_ki * halfex * dt
            self.integralFBy += two_ki * halfey * dt
            self.integralFBz += two_ki * halfez * dt
            gx += self.integralFBx
            gy += self.integralFBy
            gz += self.integralFBz
        else:
            self.integralFBx = 0.0
            self.integralFBy = 0.0
            self.integralFBz = 0.0

        gx += two_kp * halfex
        gy += two_kp * halfey
        gz += two_kp * halfez

        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt

        qa = self.q0
        qb = self.q1
        qc = self.q2

        self.q0 += -qb * gx - qc * gy - self.q3 * gz
        self.q1 += qa * gx + qc * gz - self.q3 * gy
        self.q2 += qa * gy - qb * gz + self.q3 * gx
        self.q3 += qa * gz + qb * gy - qc * gx

        norm = math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        if norm == 0.0:
            return
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

    # -----------------------------
    # 主更新函数
    # -----------------------------
    def update(self):
        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, self.last_time_us) / 1000000.0
        self.last_time_us = now_us

        # dt 保护：ticker 偶尔卡顿时，不允许一次积分一个很大的 dt。
        dt_min = self._get_cfg('DT_MIN', 0.001)
        dt_max = self._get_cfg('DT_MAX', 0.020)
        if dt < dt_min:
            return
        if dt > dt_max:
            dt = dt_max

        acc_to_g = self._get_cfg('ACC_LSB_TO_G', 1.0 / 4096.0)
        gyro_to_deg = self._get_cfg('GYRO_LSB_TO_DEG', 1.0 / 16.4)

        # 注意：这里沿用你旧代码 az 取负的坐标处理方式。
        ax = self.imu_data[0] * acc_to_g
        ay = self.imu_data[1] * acc_to_g
        az = -self.imu_data[2] * acc_to_g
        acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
        self.last_acc_norm = acc_norm

        gx_raw = self.imu_data[3] * gyro_to_deg
        gy_raw = self.imu_data[4] * gyro_to_deg
        gz_raw = self.imu_data[5] * gyro_to_deg

        gx_deg = gx_raw - self.gyro_x_bias
        gy_deg = gy_raw - self.gyro_y_bias
        gz_deg = (gz_raw - self.gyro_z_bias) * self._get_cfg('SCALE_FACTOR_Z', 1.0)

        # 一阶低通滤波，降低随机噪声。alpha 越大越平滑，但响应越慢。
        alpha = self._get_cfg('GYRO_LPF_ALPHA', 0.70)
        self.gx_f = alpha * self.gx_f + (1.0 - alpha) * gx_deg
        self.gy_f = alpha * self.gy_f + (1.0 - alpha) * gy_deg
        self.gz_f = alpha * self.gz_f + (1.0 - alpha) * gz_deg

        gx_deg = self.gx_f
        gy_deg = self.gy_f
        gz_deg = self.gz_f

        # 静止判断：不能只看 gz，要看三轴角速度和加速度模长。
        acc_still_tol = self._get_cfg('STILL_ACC_TOL', 0.08)
        gyro_still_th = self._get_cfg('STILL_GYRO_TH', 0.8)
        is_still_now = (
            abs(acc_norm - 1.0) < acc_still_tol and
            abs(gx_deg) < gyro_still_th and
            abs(gy_deg) < gyro_still_th and
            abs(gz_deg) < gyro_still_th
        )

        if self.enable_dynamic_bias and is_still_now:
            self.still_count += 1
            if self.still_count > self._get_cfg('STILL_CONFIRM_COUNT', 30):
                # 用很小的步长慢慢修正零偏，避免误判静止时把真实转向吃掉。
                beta = self._get_cfg('BIAS_UPDATE_ALPHA', 0.002)
                self.gyro_x_bias = (1.0 - beta) * self.gyro_x_bias + beta * gx_raw
                self.gyro_y_bias = (1.0 - beta) * self.gyro_y_bias + beta * gy_raw
                self.gyro_z_bias = (1.0 - beta) * self.gyro_z_bias + beta * gz_raw
        else:
            self.still_count = 0

        self.is_still = self.still_count > self._get_cfg('STILL_CONFIRM_COUNT', 30)

        # 小角速度死区：静止时残留的 0.05~0.2 deg/s 会导致 yaw 长时间漂移。
        deadband = self._get_cfg('GYRO_Z_DEADBAND', 0.18)
        if abs(gz_deg) < deadband:
            gz_deg = 0.0

        self.last_gz_deg = gz_deg

        # roll/pitch 使用 Mahony，yaw 仍然主要使用 gz 积分。
        gx_rad = gx_deg * (math.pi / 180.0)
        gy_rad = gy_deg * (math.pi / 180.0)
        gz_rad = gz_deg * (math.pi / 180.0)
        self._mahony_update_6dof(gx_rad, gy_rad, gz_rad, ax, ay, az, dt)

        self.roll = math.degrees(math.atan2(
            2.0 * (self.q0 * self.q1 + self.q2 * self.q3),
            1.0 - 2.0 * (self.q1 * self.q1 + self.q2 * self.q2)
        ))
        tmp = 2.0 * (self.q0 * self.q2 - self.q3 * self.q1)
        if tmp > 1.0:
            tmp = 1.0
        elif tmp < -1.0:
            tmp = -1.0
        self.pitch = math.degrees(math.asin(tmp))

        # 对地面车来说，yaw 直接积分 Z 轴角速度更可控，便于调 deadband / bias / scale。
        self.yaw = self._wrap_angle(self.yaw + gz_deg * dt)

    def get_yaw(self):
        return self.yaw

    def get_euler(self):
        return self.roll, self.pitch, self.yaw

    def get_debug(self):
        return {
            'yaw': self.yaw,
            'gz': self.last_gz_deg,
            'bias_z': self.gyro_z_bias,
            'acc_norm': self.last_acc_norm,
            'still': self.is_still,
        }
