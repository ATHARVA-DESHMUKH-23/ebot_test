import math
import time

class KalmanAngle:
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

        self.angle = 0.0
        self.bias = 0.0

        self.P = [[0.0, 0.0],
                  [0.0, 0.0]]

    def update(self, new_angle, new_rate, dt):
        # Predict
        rate = new_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Update
        S = self.P[0][0] + self.R_measure
        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        y = new_angle - self.angle
        self.angle += K0 * y
        self.bias += K1 * y

        P00 = self.P[0][0]
        P01 = self.P[0][1]

        self.P[0][0] -= K0 * P00
        self.P[0][1] -= K0 * P01
        self.P[1][0] -= K1 * P00
        self.P[1][1] -= K1 * P01

        return self.angle
