import numpy as np


class PID:
    def __init__(self, kp, ki, kd, rate, limit=5000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.rate = rate
        self.prev_error = 0
        self.integ = 0
        self.limit = limit

    def reset(self):
        self.prev_error = 0
        self.integ = 0

    def update_pid(self, current, desired):
        self.current = current
        self.desired = desired
        self.error = self.calc_error()

    def calc_error(self):
        return self.desired - self.current

    def calc_prop(self):
        return self.kp * self.error

    def calc_integral(self):
        self.integ += self.ki * self.error * (1 / self.rate)
        return np.clip(self.integ, -self.limit, self.limit)

    def calc_deriv(self):
        deriv = (self.error - self.prev_error) / (1 / self.rate)
        return self.kd * deriv

    def next_action(self, current, desired):
        self.update_pid(current, desired)
        next_action = self.calc_prop() + self.calc_integral() + self.calc_deriv()
        self.prev_error = self.error
        return next_action
