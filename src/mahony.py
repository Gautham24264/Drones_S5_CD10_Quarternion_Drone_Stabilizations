"""Mahony complementary filter — IMU fusion into a unit quaternion.

Corrects gyroscope drift using the accelerometer's measurement of gravity
(the 'down' direction), as described in the project brief:

    e = v_acc × v_est
    ω_corrected = ω + Kp_f e + Ki_f ∫ e dt
"""

from __future__ import annotations

import numpy as np

from . import quaternion as Q


class MahonyFilter:
    def __init__(self, kp: float = 2.0, ki: float = 0.05, q0: np.ndarray | None = None):
        self.kp = kp
        self.ki = ki
        self.q = Q.normalize(q0 if q0 is not None else np.array([1.0, 0.0, 0.0, 0.0]))
        self.integral = np.zeros(3)

    def gravity_body(self) -> np.ndarray:
        """Estimated accelerometer direction at rest: R(q)ᵀ [0, 0, 1]."""
        return Q.rotation_matrix(self.q).T @ np.array([0.0, 0.0, 1.0])

    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> np.ndarray:
        gyro = np.asarray(gyro, dtype=float).reshape(3)
        accel = np.asarray(accel, dtype=float).reshape(3)
        an = np.linalg.norm(accel)
        omega = gyro.copy()
        if an > 1e-6:
            v_acc = accel / an
            v_est = self.gravity_body()
            err = np.cross(v_acc, v_est)
            self.integral = self.integral + err * dt
            omega = gyro + self.kp * err + self.ki * self.integral
        self.q = Q.integrate(self.q, omega, dt)
        return self.q
