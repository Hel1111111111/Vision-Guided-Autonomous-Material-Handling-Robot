# encoder.py
# 编码器管理模块：保留旧版引脚和 capture_div=1，同时提供新框架接口。

from smartcar import encoder
from config import PinConfig


class EncoderManager:
    def __init__(self):
        self.enc_a = encoder(PinConfig.ENCODER_A[0], PinConfig.ENCODER_A[1], capture_div=1)
        self.enc_b = encoder(PinConfig.ENCODER_B[0], PinConfig.ENCODER_B[1], capture_div=1)
        self.enc_c = encoder(PinConfig.ENCODER_C[0], PinConfig.ENCODER_C[1], capture_div=1)

        self.raw_a = 0
        self.raw_b = 0
        self.raw_c = 0

    def get_raw_speeds(self):
        self.raw_a = self.enc_a.get()
        self.raw_b = self.enc_b.get()
        self.raw_c = self.enc_c.get()
        return self.raw_a, self.raw_b, self.raw_c

    def get_encoder_objects(self):
        # 给 ticker.capture_list(...) 使用
        return self.enc_a, self.enc_b, self.enc_c
