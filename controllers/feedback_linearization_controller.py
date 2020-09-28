import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp, 0.21, 0.05)
        self.kd = np.diag((1, 1)) * [2, -1]
        self.kp = np.diag((1, 1)) * [-2, 1.5]

    def calculate_control(self, x, q_d, q_d_dot, q_dd_dot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        q0_dot = x[2:].reshape(-1, 1)
        q0 = x[:2].reshape(-1, 1)
        q_d_dot = q_d_dot.reshape(-1, 1)
        q_d = q_d.reshape(-1, 1)

        v = q_dd_dot + self.kd.dot(q0_dot - q_d_dot) + self.kp.dot(q0 - q_d)
        z = self.model.M(x).dot(v) + self.model.C(x).dot(x[2:].reshape(-1, 1))

        return z
        #return NotImplementedError()
