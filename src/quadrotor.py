"""6-DOF rigid-body quadrotor in quaternion form (ENU, z-up).

Translational and rotational dynamics follow Shevidi & Hashim (2024)
with the more presentation-friendly ENU convention:

    v̇ = g_vec + R(q) [0, 0, F/m]
    q̇ = 1/2 q ⊗ [0, ω]
    J ω̇ = −ω × (J ω) + τ
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import quaternion as Q


@dataclass
class QuadrotorParams:
    mass: float = 1.0
    inertia: np.ndarray | None = None
    arm_length: float = 0.2
    gravity: float = 9.81
    max_thrust: float = 25.0
    max_torque: float = 4.0

    def __post_init__(self) -> None:
        if self.inertia is None:
            self.inertia = np.diag([0.012, 0.012, 0.021])
        else:
            self.inertia = np.asarray(self.inertia, dtype=float).reshape(3, 3)


class Quadrotor:
    def __init__(
        self,
        params: QuadrotorParams | None = None,
        position: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        quaternion: np.ndarray | None = None,
        omega: np.ndarray | None = None,
    ):
        self.p = params or QuadrotorParams()
        self.position = np.asarray(position if position is not None else [0.0, 0.0, 1.0], dtype=float)
        self.velocity = np.asarray(velocity if velocity is not None else [0.0, 0.0, 0.0], dtype=float)
        self.q = Q.normalize(quaternion if quaternion is not None else np.array([1.0, 0.0, 0.0, 0.0]))
        self.omega = np.asarray(omega if omega is not None else [0.0, 0.0, 0.0], dtype=float)
        self.J_inv = np.linalg.inv(self.p.inertia)
        self.last_accel = np.zeros(3)

    @property
    def hover_thrust(self) -> float:
        return self.p.mass * self.p.gravity

    def step(
        self,
        thrust: float,
        torque: np.ndarray,
        dt: float,
        wind: np.ndarray | None = None,
        attitude_only: bool = False,
    ) -> np.ndarray:
        thrust = float(np.clip(thrust, 0.0, self.p.max_thrust))
        torque = np.clip(np.asarray(torque, dtype=float).reshape(3), -self.p.max_torque, self.p.max_torque)
        wind = np.zeros(3) if wind is None else np.asarray(wind, dtype=float).reshape(3)

        R = Q.rotation_matrix(self.q)
        accel = np.array([0.0, 0.0, -self.p.gravity]) + R @ np.array([0.0, 0.0, thrust / self.p.mass]) + wind
        self.last_accel = accel.copy()
        if attitude_only:
            self.velocity[:] = 0.0
        else:
            self.velocity = self.velocity + accel * dt
            self.position = self.position + self.velocity * dt
            if self.position[2] < 0.0:
                self.position[2] = 0.0
                if self.velocity[2] < 0.0:
                    self.velocity[2] = 0.0

        Jw = self.p.inertia @ self.omega
        omega_dot = self.J_inv @ (-np.cross(self.omega, Jw) + torque)
        self.omega = self.omega + omega_dot * dt
        self.q = Q.integrate(self.q, self.omega, dt)
        return self.last_accel
