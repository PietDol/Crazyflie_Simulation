from collections import deque


class PID:
    def __init__(self, u0: float, kp: float, kd: float, ki: float, dt: float):
        self.u0 = u0
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt

        self.F = [kp + ki * dt + kd / dt, -kp - 2 * kd / dt, kd / dt]

        self.window = None
        self.u = None

    def reset(self):
        self.u = self.u0
        self.window = deque(maxlen=3)

    def next_action(self, y: float, ref: float = 0.0):
        # Add error
        self.window.appendleft(ref - y)

        # Calculate action
        for idx, e in enumerate(self.window):
            self.u += self.F[idx] * e

        print(self.u)

        return self.u


# Try some things
gains = [1.0, 0.5, 0.0]
controller = PID(u0=0.0, kp=gains[0], kd=gains[1], ki=gains[2], dt=1/20)
controller.reset()
controller.next_action(4)
controller.next_action(3)
controller.next_action(2)
controller.next_action(1)
controller.next_action(0.5)
