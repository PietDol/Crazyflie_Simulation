import numpy as np


class PID:
    """ The PID controller """
    def __init__(self, kp, ki, kd, rate, limit=5000):
        """ Initiate a PID controller object

        :param kp: The proportional constant of a PID controller
        :param ki: The integral constant of a PID controller
        :param kd: The derivative constant of a PID controller
        :param rate: To rate at which the controller is called to calculate the time step
        :param limit: The integral limit to prevent integral wind-up
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.rate = rate
        self.prev_error = 0
        self.integ = 0
        self.limit = limit

    def reset(self):
        """ To reset the PID controller as if it was just created. """
        self.prev_error = 0
        self.integ = 0

    def update_pid(self, current, desired):
        """ Update the current and desired value """
        self.current = current
        self.desired = desired
        self.error = self.calc_error()

    def calc_error(self):
        """ Calculate the error """
        return self.desired - self.current

    def calc_prop(self):
        """ Calculate the proportional part of the output """
        return self.kp * self.error

    def calc_integral(self):
        """ Calculate the integral part of the output """
        self.integ += self.ki * self.error * (1 / self.rate)
        return np.clip(self.integ, -self.limit, self.limit)

    def calc_deriv(self):
        """ Calculate the derivative part of the output """
        deriv = (self.error - self.prev_error) / (1 / self.rate)
        return self.kd * deriv

    def next_action(self, current, desired):
        """ Function that is called every timestep to calculate the output
        The calculations are performed according to formula 1 in the paper on the Github.

        :param current: The current value
        :param desired: The reference value

        :return next_action: The output
        """
        self.update_pid(current, desired)
        next_action = self.calc_prop() + self.calc_integral() + self.calc_deriv()
        self.prev_error = self.error
        return next_action
