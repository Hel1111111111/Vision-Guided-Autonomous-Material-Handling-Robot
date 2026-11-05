# motor.py
# 电机管理模块：在新框架中复现旧版 motor_ctrl.py 的电机初始化、move 运动学、速度倍率和 PID 输出链路。

import math
from seekfree import MOTOR_CONTROLLER
from config import MotorConfig, PIDConfig
from pid import IncrementalPID


class MotorManager:
    def __init__(self):
        # 旧版已验证电机初始化方式
        self.motor_1 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, MotorConfig.PWM_FREQ, duty=0, invert=False)
        self.motor_2 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, MotorConfig.PWM_FREQ, duty=0, invert=True)
        self.motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5,   MotorConfig.PWM_FREQ, duty=0, invert=False)
        self.motor_4 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7,   MotorConfig.PWM_FREQ, duty=0, invert=True)

        self.pid_a = IncrementalPID(PIDConfig.A_KP, PIDConfig.A_KI, PIDConfig.A_KD, MotorConfig.MAX_DUTY)
        self.pid_b = IncrementalPID(PIDConfig.B_KP, PIDConfig.B_KI, PIDConfig.B_KD, MotorConfig.MAX_DUTY)
        self.pid_c = IncrementalPID(PIDConfig.C_KP, PIDConfig.C_KI, PIDConfig.C_KD, MotorConfig.MAX_DUTY)

        self.target_a = 0
        self.target_b = 0
        self.target_c = 0

        self.raw_a = 0
        self.raw_b = 0
        self.raw_c = 0

        self.speed_a = 0
        self.speed_b = 0
        self.speed_c = 0

        self.out_a = 0
        self.out_b = 0
        self.out_c = 0

    def set_velocity(self, vx, vy, w):
        """
        旧版 mc.move(vx, vy, w) 的新框架版本。
        注意这里的 vx/vy/w 量级沿用旧版：例如 -2500、1200、800。
        """
        L = MotorConfig.ROTATE_L
        self.target_a = int(0 * vx - (2.0 / 3.0) * vy + L * w)
        self.target_b = int(-math.sqrt(3) / 3.0 * vx + (1.0 / 3.0) * vy + L * w)
        self.target_c = int( math.sqrt(3) / 3.0 * vx + (1.0 / 3.0) * vy + L * w)

    def set_wheel_targets(self, a, b, c):
        self.target_a = int(a)
        self.target_b = int(b)
        self.target_c = int(c)

    def update_from_encoder_raw(self, raw_a, raw_b, raw_c):
        self.raw_a = raw_a
        self.raw_b = raw_b
        self.raw_c = raw_c
        self.speed_a = MotorConfig.RATE_A * raw_a
        self.speed_b = MotorConfig.RATE_B * raw_b
        self.speed_c = MotorConfig.RATE_C * raw_c

    def control_loop(self):
        self.out_a = self.pid_a.update(self.target_a, self.speed_a)
        self.out_b = self.pid_b.update(self.target_b, self.speed_b)
        self.out_c = self.pid_c.update(self.target_c, self.speed_c)

        # 旧版 motorA/motorB/motorC 映射：motor_1 与 motor_2 共用 A 输出
        self.motor_1.duty(int(self.out_a))
        self.motor_2.duty(int(self.out_a))
        self.motor_3.duty(int(self.out_b))
        self.motor_4.duty(int(self.out_c))

    def stop(self):
        self.set_wheel_targets(0, 0, 0)
        self.pid_a.reset()
        self.pid_b.reset()
        self.pid_c.reset()
        self.out_a = 0
        self.out_b = 0
        self.out_c = 0
        self.motor_1.duty(0)
        self.motor_2.duty(0)
        self.motor_3.duty(0)
        self.motor_4.duty(0)

    def get_debug(self):
        return {
            'target': (self.target_a, self.target_b, self.target_c),
            'raw': (self.raw_a, self.raw_b, self.raw_c),
            'speed': (self.speed_a, self.speed_b, self.speed_c),
            'out': (self.out_a, self.out_b, self.out_c),
        }
