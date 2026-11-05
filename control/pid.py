# pid.py
# 旧版增量式 PID 的类封装版本。

class IncrementalPID:
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.err_last = 0
        self.err_last2 = 0
        self.out = 0

    def reset(self):
        self.err_last = 0
        self.err_last2 = 0
        self.out = 0

    def _limit(self, value):
        if value > self.max_out:
            return self.max_out
        if value < -self.max_out:
            return -self.max_out
        return value

    def update(self, target, measured):
        # 完全保留旧版逻辑：目标为 0 时直接清零，避免停车后积分残留。
        if target == 0:
            self.reset()
            return 0

        err = target - measured
        delta_out = (
            self.kp * (err - self.err_last)
            + self.ki * err
            + self.kd * (err - 2 * self.err_last + self.err_last2)
        )

        self.out = self._limit(self.out + delta_out)
        self.err_last2 = self.err_last
        self.err_last = err
        return self.out
