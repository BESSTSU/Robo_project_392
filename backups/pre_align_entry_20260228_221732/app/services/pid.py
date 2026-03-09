"""PID helper."""

import time


class PID:
    def __init__(self, kp: float, ki: float, kd: float, integral_max: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        self.reset()

    def reset(self) -> None:
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_ts = time.monotonic()

    def update(self, error: float) -> float:
        now = time.monotonic()
        dt = max(1e-3, now - self.last_ts)

        p = self.kp * error

        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i = self.ki * self.integral

        d = self.kd * (error - self.prev_error) / dt

        self.prev_error = error
        self.last_ts = now
        return p + i + d
