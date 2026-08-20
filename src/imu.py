"""Simulated MPU-6050-class IMU: gyroscope and accelerometer with noise/bias."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import quaternion as Q


@dataclass
class IMUNoise:
    gyro_std: float = 0.01
    accel_std: float = 0.15
    gyro_bias: np.ndarray | None = None
    seed: int = 7


class IMU:
    def __init__(self, noise: IMUNoise | None = None, gravity: float = 9.81):
        self.noise = noise or IMUNoise()
        self.gravity = gravity
        self.rng = np.random.default_rng(self.noise.seed)
        self.bias = (
            np.asarray(self.noise.gyro_bias, dtype=float).reshape(3)
            if self.noise.gyro_bias is not None
            else np.array([0.008, -0.006, 0.004])
        )

    def measure(
        self,
        q: np.ndarray,
        omega: np.ndarray,
        accel_inertial: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (gyro, accel) in the body frame.

        Accelerometer reports specific force: Rᵀ (a − g_vec), so at rest
        it reads +g along body-z when the vehicle is level (ENU).
        """
        R = Q.rotation_matrix(q)
        g_vec = np.array([0.0, 0.0, -self.gravity])
        accel_true = R.T @ (accel_inertial - g_vec)
        gyro = omega + self.bias + self.rng.normal(0.0, self.noise.gyro_std, size=3)
        accel = accel_true + self.rng.normal(0.0, self.noise.accel_std, size=3)
        return gyro, accel
