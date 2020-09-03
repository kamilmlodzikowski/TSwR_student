import numpy as np
from .controller import Controller


class PDDecentralizedController(Controller):
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd

    def calculate_control(self, q, q_dot, q_d, q_d_dot, q_d_ddot):
        ### TODO: Please implement me
        e = q - q_d
        e_dot = q_dot - q_d_dot

        return self.kp*e + self.kd*e_dot
