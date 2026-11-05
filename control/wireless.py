# wireless.py
# 双车无线通信模块：保留旧版发送 AA55 协议和接收 AA66 mov 协议。

from array import array
from seekfree import WIRELESS_UART
from config import WirelessConfig


class WirelessManager:
    def __init__(self):
        self.wireless = WIRELESS_UART(WirelessConfig.BAUD)
        self.send_buffer = array('B', [0] * WirelessConfig.SEND_FRAME_LEN)
        self.receive_buffer = array('B', [0])
        self.rx_state = 0
        self.rx_index = 0
        # rx_data 要同时给 AA66(mov, 9字节) 和 AA55(velocity, 10字节) 两种帧复用，按较大的分配。
        self.rx_data = [0] * max(WirelessConfig.RX_PAYLOAD_LEN, WirelessConfig.VELOCITY_RX_PAYLOAD_LEN)
        self.mov = 0
        self.velocity = None

    def send_velocity(self, a, b, c, ctrl, class_id, weight_mode=0):
        self.send_buffer[0] = 0xAA
        self.send_buffer[1] = 0x55
        self.send_buffer[2] = class_id & 0xFF
        self.send_buffer[3] = ctrl & 0xFF
        self.send_buffer[4] = weight_mode & 0xFF

        self.send_buffer[5] = int(a) & 0xFF
        self.send_buffer[6] = (int(a) >> 8) & 0xFF
        self.send_buffer[7] = int(b) & 0xFF
        self.send_buffer[8] = (int(b) >> 8) & 0xFF
        self.send_buffer[9] = int(c) & 0xFF
        self.send_buffer[10] = (int(c) >> 8) & 0xFF

        self.send_buffer[11] = (
            self.send_buffer[4] + self.send_buffer[5] +
            self.send_buffer[6] + self.send_buffer[7] +
            self.send_buffer[8] + self.send_buffer[9] +
            self.send_buffer[10]
        ) & 0xFF
        self.send_buffer[12] = 0xFE
        self.wireless.send_bytearray(self.send_buffer, WirelessConfig.SEND_FRAME_LEN)

    def receive_once(self):
        # 旧版每次读取 1 字节，这里也保留，外层可以多调用几次。
        # 修复：receive_bytearray 无新数据时返回 0，此时 receive_buffer[0] 是上一次的陈旧值，
        # 之前没判断返回值就直接拿去喂状态机，会把陈旧字节当新数据触发帧同步错乱。
        n = self.wireless.receive_bytearray(self.receive_buffer, 1)
        if n == 0:
            return self.mov
        b = self.receive_buffer[0]

        if self.rx_state == 0:
            if b == 0xAA:
                self.rx_state = 1
        elif self.rx_state == 1:
            if b == 0x66:
                self.rx_state = 2
                self.rx_index = 0
            else:
                self.rx_state = 0
        elif self.rx_state == 2:
            self.rx_data[self.rx_index] = b
            self.rx_index += 1
            if self.rx_index >= WirelessConfig.RX_PAYLOAD_LEN:
                if self.rx_data[8] != 0xFE:
                    self.rx_state = 0
                    return self.mov

                check = (
                    self.rx_data[0] +
                    self.rx_data[1] + self.rx_data[2] +
                    self.rx_data[3] + self.rx_data[4] +
                    self.rx_data[5] + self.rx_data[6]
                ) & 0xFF

                if check == self.rx_data[7]:
                    self.mov = self.rx_data[0]

                self.rx_state = 0
        return self.mov

    def receive_many(self, count=20):
        for _ in range(count):
            self.receive_once()
        return self.mov

    def receive_velocity_once(self):
        """从车侧解析主车 send_velocity 发出的 AA55 速度帧，同样带返回长度判断的修复。"""
        n = self.wireless.receive_bytearray(self.receive_buffer, 1)
        if n == 0:
            return None
        b = self.receive_buffer[0]

        if self.rx_state == 0:
            if b == 0xAA:
                self.rx_state = 1
        elif self.rx_state == 1:
            if b == 0x55:
                self.rx_state = 3
                self.rx_index = 0
            else:
                self.rx_state = 0
        elif self.rx_state == 3:
            self.rx_data[self.rx_index] = b
            self.rx_index += 1
            if self.rx_index >= WirelessConfig.VELOCITY_RX_PAYLOAD_LEN:
                self.rx_state = 0
                if self.rx_data[9] != 0xFE:
                    return None

                check = (
                    self.rx_data[2] + self.rx_data[3] +
                    self.rx_data[4] + self.rx_data[5] +
                    self.rx_data[6] + self.rx_data[7]
                ) & 0xFF
                if check != self.rx_data[8]:
                    return None

                class_id = self.rx_data[0]
                ctrl = self.rx_data[1]
                a = self.rx_data[2] | (self.rx_data[3] << 8)
                b_val = self.rx_data[4] | (self.rx_data[5] << 8)
                c = self.rx_data[6] | (self.rx_data[7] << 8)
                if a > 32767:
                    a -= 65536
                if b_val > 32767:
                    b_val -= 65536
                if c > 32767:
                    c -= 65536

                self.velocity = (class_id, ctrl, a, b_val, c)
                return self.velocity
        return None

    def receive_velocity_many(self, count=20):
        """按次数轮询，返回本轮里最后一次成功解析出的速度帧，没有则返回 None。"""
        result = None
        for _ in range(count):
            got = self.receive_velocity_once()
            if got is not None:
                result = got
        return result

    def send_debug_str(self, text):
        self.wireless.send_str(text)

    def send_oscilloscope(self, *values):
        """转发到逐飞库 WIRELESS_UART.send_oscilloscope，最多8个浮点通道，配合"逐飞助手-虚拟示波器"使用。"""
        self.wireless.send_oscilloscope(*values)

