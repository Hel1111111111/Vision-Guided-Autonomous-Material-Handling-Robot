import time

from system import CarSystem


sys = CarSystem()

def start_tickers(self):
    self.pit_control = ticker(TickerConfig.CONTROL_ID)
    self.pit_control.callback(self._control_loop)

    enc_a, enc_b, enc_c = self.encoder.get_encoder_objects()
    self.pit_control.capture_list(
        enc_a,
        enc_b,
        enc_c
    )

    self.pit_control.start(10)

    self.pit_vision = ticker(TickerConfig.VISION_ID)
    self.pit_vision.callback(self._vision_loop)
    self.pit_vision.start(10)
    

def _control_loop(self, ticker_instance):
    raw_a, raw_b, raw_c = self.encoder.get_raw_speeds()

    self.motor.update_from_encoder_raw(
        raw_a,
        raw_b,
        raw_c
    )

    self.motor.control_loop()

    # 暂时关闭
    # self.odo.update()
# 这个测试不需要IMU标定
# sys.calibrate()

# 关闭双车速度广播
sys.enable_velocity_broadcast = False

# 使用精简ticker
sys.start_tickers()

CLASS_ID = 0

sys.vision.send_switch_command(CLASS_ID)
time.sleep_ms(500)

last_print = time.ticks_ms()

while True:
    # 先从低速度开始
    sys.motor.set_velocity(-500, 0, 0)

    if time.ticks_diff(
        time.ticks_ms(),
        last_print
    ) >= 500:
        last_print = time.ticks_ms()

        sys.wireless.send_debug_str(
            'valid:{} err:{:.3f} dist:{:.3f}\r\n'.format(
                sys.vision.is_valid(),
                sys.vision.get_error_x(),
                sys.vision.get_distance_hint()
            )
        )

    time.sleep_ms(1)