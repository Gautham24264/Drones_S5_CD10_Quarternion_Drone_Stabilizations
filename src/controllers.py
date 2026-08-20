"""Attitude and cascaded position controllers.

The inner loop is the project contribution: quaternion error → PID.
The outer loop is a simplified stand-in for the paper's adaptive
backstepping position controller — enough to fly a 3D trajectory
without the full sliding-mode law.
"""

from __future__ import annotations

import numpy as np

from . import quaternion as Q
from .pid import PID3


class QuaternionAttitudePID:
    """u = Kp q_e_vec + Ki ∫ q_e_vec − Kd ω  (rate damping on gyro)."""

    def __init__(self, kp: float = 4.5, ki: float = 0.15, kd: float = 0.55):
        self.pid = PID3(kp=kp, ki=ki, kd=0.0)
        self.kd = kd

    def reset(self) -> None:
        self.pid.reset()

    def update(self, q_desired: np.ndarray, q_current: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
        err = Q.vector_error(q_desired, q_current)
        return self.pid.update(err, dt, derivative=np.zeros(3)) - self.kd * np.asarray(omega, dtype=float)


class EulerAttitudePID:
    """Roll/pitch/yaw PID inverted through the Euler kinematic Jacobian.

    Body rates and Euler rates are related by a matrix E(θ) that loses rank
    at pitch = ±90°. Inverting it — which Euler-based controllers must do —
    sends the torque command to infinity. That is gimbal lock in the loop.
    """

    def __init__(self, kp: float = 4.5, ki: float = 0.15, kd: float = 0.55):
        self.pid = PID3(kp=kp, ki=ki, kd=0.0)
        self.kd = kd

    def reset(self) -> None:
        self.pid.reset()

    def update(self, q_desired: np.ndarray, q_current: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
        eul_d = Q.to_euler(q_desired)
        eul = Q.to_euler(q_current)
        err = _wrap_angle(eul_d - eul)
        u = self.pid.update(err, dt, derivative=np.zeros(3))
        roll, pitch, _ = eul
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        E = np.array(
            [
                [1.0, 0.0, -sp],
                [0.0, cr, sr * cp],
                [0.0, -sr, cr * cp],
            ]
        )
        try:
            tau = np.linalg.solve(E, u)
        except np.linalg.LinAlgError:
            tau = np.linalg.pinv(E) @ u
        return tau - self.kd * np.asarray(omega, dtype=float)


class CascadedPositionController:
    """Outer P/D on position → desired thrust vector → quaternion setpoint."""

    def __init__(
        self,
        mass: float = 1.0,
        gravity: float = 9.81,
        kp_pos: float = 2.2,
        kd_pos: float = 2.0,
        max_tilt: float = 0.55,
        yaw_desired: float = 0.0,
    ):
        self.mass = mass
        self.gravity = gravity
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.max_tilt = max_tilt
        self.yaw_desired = yaw_desired

    def update(
        self,
        p_desired: np.ndarray,
        p: np.ndarray,
        v: np.ndarray,
        v_desired: np.ndarray | None = None,
        a_desired: np.ndarray | None = None,
    ) -> tuple[float, np.ndarray]:
        e = np.asarray(p_desired, dtype=float) - np.asarray(p, dtype=float)
        v_d = np.zeros(3) if v_desired is None else np.asarray(v_desired, dtype=float)
        a_ff = np.zeros(3) if a_desired is None else np.asarray(a_desired, dtype=float)
        a_des = (
            self.kp_pos * e
            + self.kd_pos * (v_d - np.asarray(v, dtype=float))
            + a_ff
            + np.array([0.0, 0.0, self.gravity])
        )
        # Limit horizontal acceleration so the drone cannot command 90° instantly.
        horiz = a_des[:2]
        max_h = self.gravity * np.tan(self.max_tilt)
        n = np.linalg.norm(horiz)
        if n > max_h:
            horiz = horiz * (max_h / n)
            a_des = np.array([horiz[0], horiz[1], a_des[2]])
        a_des[2] = max(a_des[2], 0.3 * self.gravity)
        thrust = self.mass * float(np.linalg.norm(a_des))
        q_des = Q.from_two_vectors_yaw(a_des, self.yaw_desired)
        return thrust, q_des


def _wrap_angle(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi
