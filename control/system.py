# system.py
# 系统装配与 ticker 管理：把 encoder/motor/vision/imu/wireless 的周期任务统一放这里。

from machine import Pin
from smartcar import ticker
from config import PinConfig, TickerConfig
from encoder import EncoderManager
from motor import MotorManager
from vision import VisionManager
from imu import IMUManager
from wireless import WirelessManager
from odometry import Odometry
import math


class CarSystem:
    def __init__(self):
        self.led = Pin(PinConfig.LED, Pin.OUT, value=True)
        self.switch2 = Pin(PinConfig.SWITCH2, Pin.IN, pull=Pin.PULL_UP_47K)
        self.switch2_initial = self.switch2.value()

        self.encoder = EncoderManager()
        self.motor = MotorManager()
        self.vision = VisionManager()
        self.imu = IMUManager()
        self.wireless = WirelessManager()
        self.odo = Odometry(self.motor, self.imu)

        self.ctrl = 1
        self.wireless_cnt = 0
        self.time_cnt=0
        self.err_c=0
        self.realerr_c=0,
        self.state_line='NO_USE'
        # 单车调试/标定时关掉这个开关，_wireless_loop 就不再发送 AA55 二进制协同帧，
        # 无线通道上只剩文本调试信息，避免二进制字节把文本调试信息挤成乱码。
        self.enable_velocity_broadcast =True

        # 关闭后广播给车2的目标速度强制为0,0,0（车1自身运动不受影响），
        # 用于让车2的前馈分量归零，只看纠偏 trim。
        self.enable_velocity_feedforward = True

        # 随AA55帧广播给车2的跟随权重模式号，车2按此号在FollowConfig.WEIGHT_PRESETS
        # 里查前馈/纠偏权重组合。0=normal（默认）。由task_runner按任务阶段设置。
        self.velocity_weight_mode = 0

        self.pit_wireless = None
        self.pit_control = None
        self.pit_vision = None
        self.pit_imu = None

    def calibrate(self):
        print('IMU calibrating, keep car still...')
        self.imu.calibrate()
        print('IMU calibrate done')

    def reset_odometry(self, x0=0.0, y0=0.0, yaw0=0.0):
        """在发车前、已知车头方向和起点坐标时调用，把里程计原点和 IMU 航向零点对齐。"""
        self.odo.reset(x0, y0, yaw0)

    def start_tickers(self):
        self.pit_wireless = ticker(TickerConfig.WIRELESS_ID)
        self.pit_wireless.callback(self._wireless_loop)
        self.pit_wireless.start(TickerConfig.WIRELESS_PERIOD_MS)

        self.pit_control = ticker(TickerConfig.CONTROL_ID)
        self.pit_control.callback(self._control_loop)
        enc_a, enc_b, enc_c = self.encoder.get_encoder_objects()
        self.pit_control.capture_list(enc_a, enc_b, enc_c)
        self.pit_control.start(TickerConfig.CONTROL_PERIOD_MS)

        self.pit_vision = ticker(TickerConfig.VISION_ID)
        self.pit_vision.callback(self._vision_loop)
        self.pit_vision.start(TickerConfig.VISION_PERIOD_MS)

        self.pit_imu = ticker(TickerConfig.IMU_ID)
        self.pit_imu.callback(self._imu_loop)
        self.pit_imu.capture_list(self.imu.get_imu_object())
        self.pit_imu.start(TickerConfig.IMU_PERIOD_MS)

    def _control_loop(self, ticker_instance):
        raw_a, raw_b, raw_c = self.encoder.get_raw_speeds()
        self.motor.update_from_encoder_raw(raw_a, raw_b, raw_c)
        self.motor.control_loop()
        self.odo.update()
    
    def _imu_loop(self, ticker_instance):
        self.imu.update()

    def _vision_loop(self, ticker_instance):
        self.vision.poll_uart()
        self.err_c=self.vision.get_error_x()*abs(math.cos(self.imu.get_yaw()))
        self.realerr_c=self.vision.get_realdist_w()*abs(math.cos(self.imu.get_yaw()))

    def _wireless_loop(self, ticker_instance):
        class_id = self.vision.get_class_id()
        if self.enable_velocity_broadcast:
            if self.enable_velocity_feedforward:
                send_a, send_b, send_c = self.motor.target_a, self.motor.target_b, self.motor.target_c
            else:
                send_a, send_b, send_c = 0, 0, 0
            self.wireless.send_velocity(
                send_a,
                send_b,
                send_c,
                self.ctrl,
                class_id,
                self.velocity_weight_mode
            )
        '''self.wireless.receive_many(20)'''

        self.wireless_cnt += 1
        if self.wireless_cnt >= 75:
            self.wireless_cnt = 0
            self.time_cnt+=1

            msg = 'time:{}\r\nline:{}\r\nid:{}\r\nself.vision.yaw:{:.2f}\r\nerrc:{:.3f}\r\nrealerrc:{:.3f}\r\nerr:{:.3f}\r\nrealerr:{:.3f}\r\ndist:{:.2f}\r\nvalid:{}\r\nodo_x:{:.1f}\r\nodo_y:{:.1f}\r\nodo_d:{:.1f}\r\n'.format(
                self.time_cnt,
                self.state_line,
                self.vision.get_class_id(),
                self.imu.get_yaw(),
                self.err_c,
                self.realerr_c,
                self.vision.get_error_x(),
                self.vision.get_realdist_w(),
                self.vision.get_distance_hint(),
                self.vision.is_valid(),
                self.odo.x,
                self.odo.y,
                self.odo.get_distance_from_origin()
            )
            '''msg = 'err:{:.3f}\r\ndist:{:.2f}\r\nvalid:{}\r\nta:{:.2f}\r\ntb:{:.3f}\r\ntc:{:.2f}\r\nspda:{}\r\nspdb:{:.1f}\r\nspdc:{:.1f}\r\nouta:{:.1f}\r\noutb:{:.1f}\r\noutc:{:.1f}\r\n'.format(
            self.vision.get_error_x(),
            self.vision.get_distance_hint(),
            self.vision.is_valid(),
            self.motor.target_a,
            self.motor.target_b,
            self.motor.target_c,
            self.motor.speed_a,
            self.motor.speed_b,
            self.motor.speed_c,
            self.motor.out_a,
            self.motor.out_b,
            self.motor.out_c
            )'''
            self.wireless.send_debug_str(msg)

    def send_position_debug(self):
        """立即通过无线串口发送一次里程计位置/位移，用于脱离USB线时查看移动距离。"""
        x, y = self.odo.get_position()
        msg = 'odo_x:{:.1f}\r\nodo_y:{:.1f}\r\nodo_d:{:.1f}\r\n'.format(
            x, y, self.odo.get_distance_from_origin()
        )
        self.wireless.send_debug_str(msg)

    def stop(self):
        self.motor.stop()

    def should_stop_by_switch(self):
        return self.switch2.value() != self.switch2_initial

    def heartbeat(self):
        self.led.value(not self.led.value())

