import numpy as np

class MadgwickAHRS:
    def __init__(self, sampleperiod=1/256, beta=0.1):
        self.sampleperiod = sampleperiod
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def update_imu(self, gx, gy, gz, ax, ay, az):
        q1, q2, q3, q4 = self.q
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        norm = 1 / norm
        ax *= norm
        ay *= norm
        az *= norm

        f1 = 2 * (q2 * q4 - q1 * q3) - ax
        f2 = 2 * (q1 * q2 + q3 * q4) - ay
        f3 = 1 - 2 * (q2 * q2 + q3 * q3) - az
        J_11or24 = 2 * q3
        J_12or23 = 2 * q4
        J_13or22 = 2 * q1
        J_14or21 = 2 * q2
        J_32 = 2 * J_14or21
        J_33 = 2 * J_11or24

        step = np.array([
            J_14or21 * f2 - J_11or24 * f1,
            J_12or23 * f1 + J_13or22 * f2 - J_32 * f3,
            J_12or23 * f2 - J_33 * f3 - J_14or21 * f1,
            J_13or22 * f1 + J_11or24 * f2
        ])
        step = step / np.linalg.norm(step)
        q_dot = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
            q1 * gx + q3 * gz - q4 * gy,
            q1 * gy - q2 * gz + q4 * gx,
            q1 * gz + q2 * gy - q3 * gx
        ]) - self.beta * step

        self.q += q_dot * self.sampleperiod
        self.q = self.q / np.linalg.norm(self.q)
