"""Decoupled 3-axis PID with optional integral clamp."""

from __future__ import annotations

import numpy as np


class PID3:
    def __init__(
        self,
        kp: float | np.ndarray,
        ki: float | np.ndarray,
        kd: float | np.ndarray,
        i_limit: float = 0.4,
    ):
        self.kp = np.broadcast_to(np.asarray(kp, dtype=float), (3,)).copy()
        self.ki = np.broadcast_to(np.asarray(ki, dtype=float), (3,)).copy()
        self.kd = np.broadcast_to(np.asarray(kd, dtype=float), (3,)).copy()
        self.i_limit = i_limit
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)
        self.has_prev = False

    def reset(self) -> None:
        self.integral[:] = 0.0
        self.prev_error[:] = 0.0
        self.has_prev = False

    def update(self, error: np.ndarray, dt: float, derivative: np.ndarray | None = None) -> np.ndarray:
        error = np.asarray(error, dtype=float).reshape(3)
        self.integral = np.clip(self.integral + error * dt, -self.i_limit, self.i_limit)
        if derivative is None:
            if self.has_prev and dt > 0:
                derivative = (error - self.prev_error) / dt
            else:
                derivative = np.zeros(3)
        self.prev_error = error.copy()
        self.has_prev = True
        return self.kp * error + self.ki * self.integral + self.kd * derivative
