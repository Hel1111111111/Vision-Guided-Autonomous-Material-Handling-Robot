# odometry.py
# 惯导定位模块：编码器测速 + IMU 航向融合，积分出全局坐标 (x, y)。
#
# 车体坐标系与 motor.set_velocity(vx, vy, w) 完全一致：yaw=0 时车头方向对应的
# vx/vy 正方向，就是 set_velocity 里 vx/vy 的正方向。世界坐标系在 reset() 时与
# 车体坐标系重合，之后随小车转动而分离。
#可能的改进？
 # 如果要做更紧耦合的融合（比如用卡尔曼滤波器同时估计位置、速度和陀螺仪零偏），可以从编码器也推算 w，与陀螺仪的 w
  #做对比，用差值来在线估计陀螺仪零偏。但这需要更多的计算资源和调参工作

import math
import time
from config import OdometryConfig


class Odometry:
    def __init__(self, motor, imu):
        self.motor = motor
        self.imu = imu

        self.x = 0.0
        self.y = 0.0

        self.vx_body = 0.0
        self.vy_body = 0.0

        self.last_time_us = time.ticks_us()

    def reset(self, x0=0.0, y0=0.0, yaw0=0.0):
        """重置里程计原点。yaw0 用于同时校正 IMU 的航向零点，一般在发车前、已知车头方向时调用。"""
        self.x = x0
        self.y = y0
        self.imu.set_yaw_zero(yaw0)
        self.last_time_us = time.ticks_us()

    def update(self):
        """在 control ticker（10ms）里，紧跟 motor.update_from_encoder_raw() 之后调用。"""
        now_us = time.ticks_us()
        dt = time.ticks_diff(now_us, self.last_time_us) / 1000000.0
        self.last_time_us = now_us
        if dt <= 0.0:
            return
        if dt > OdometryConfig.DT_MAX:
            dt = OdometryConfig.DT_MAX

        sa = self.motor.speed_a
        sb = self.motor.speed_b
        sc = self.motor.speed_c

        # 三轮全向轮正解：motor.set_velocity() 逆解矩阵的逆矩阵，与 MotorConfig.ROTATE_L 保持对应。
        #自己算一下？#不对，这个是不是能够调用移动函数的计算？
        vx_raw = (math.sqrt(3) / 2.0) * (sc - sb)
        vy_raw = (sb + sc) / 2.0 - sa

        k = OdometryConfig.SPEED_TO_MMPS
        self.vx_body = vx_raw * k
        self.vy_body = vy_raw * k

        yaw_rad = math.radians(self.imu.get_yaw())
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)

        vx_world = self.vx_body * cos_y - self.vy_body * sin_y
        vy_world = self.vx_body * sin_y + self.vy_body * cos_y

        self.x += vx_world * dt
        self.y += vy_world * dt

    def get_position(self):
        return self.x, self.y

    def get_yaw(self):
        return self.imu.get_yaw()

    def get_distance_from_origin(self):
        """相对最近一次 reset() 原点的直线位移，单位 mm。"""
        return math.sqrt(self.x * self.x + self.y * self.y)

    def get_debug(self):
        return {
            'x': self.x,
            'y': self.y,
            'yaw': self.imu.get_yaw(),
            'vx_body': self.vx_body,
            'vy_body': self.vy_body,
        }
