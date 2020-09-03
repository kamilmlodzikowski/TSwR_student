import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # Use parameters from manipulators/mm_planar_2dof.py
        self.models = [ManiuplatorModel(Tp, 0.1, 0.05), ManiuplatorModel(Tp, 0.01, 0.01), ManiuplatorModel(Tp, 1.0, 0.3)]
        self.i = 0
        self.kd = np.diag((1, 1)) * [1.5, -1]
        self.kp = np.diag((1, 1)) * [-1, 2]
    def choose_model(self, x, u, x_dot):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        min = 1000000000
        max = 0
        for i, mdl in enumerate(self.models):
            x_m = self.calc_x_m(mdl, x, u)
            y = abs(x_m-x_dot)
            y2 = np.sum(y)
            if y2 < min:
                max = i
                worst = y2
        self.i = max


    def calc_x_m(self, mdl, x , u):
        inv_M = np.linalg.inv(mdl.M(x))
        zeros = np.zeros((2, 2), dtype=np.float32)

        A = np.concatenate([np.concatenate([zeros, np.eye(2)], 1),
                            np.concatenate([zeros, -inv_M @ mdl.C(x)], 1)], 0)
        b = np.concatenate([zeros, inv_M], 0)
        x_m = A @ x[:, np.newaxis] + b @ u
        return x_m


    def calculate_control(self, x, q_d, q_d_dot, q_dd_dot):
        q0_dot = x[2:].reshape(-1, 1)
        q0 = x[:2].reshape(-1, 1)
        q_d_dot = q_d_dot.reshape(-1, 1)
        q_d = q_d.reshape(-1, 1)

        v = q_dd_dot + self.kp.dot(q0 - q_d) + self.kd.dot(q0_dot - q_d_dot)  # TODO: Add Feedback
        q_dot = x[2:, np.newaxis]
        M = self.models[self.i].M(x)
        return M @ v + self.models[self.i].C(x) @ q_dot
