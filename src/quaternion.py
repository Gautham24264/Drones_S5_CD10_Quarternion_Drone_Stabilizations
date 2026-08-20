"""Unit-quaternion helpers for singularity-free attitude representation.

Follows the project brief and Shevidi & Hashim (2024): a unit quaternion
q = [q0, q1, q2, q3] with scalar-first convention and ||q|| = 1.
"""

from __future__ import annotations

import numpy as np

EPS = 1e-12


def _as_vec4(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    return q


def normalize(q: np.ndarray) -> np.ndarray:
    q = _as_vec4(q)
    n = np.linalg.norm(q)
    if n < EPS:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def conjugate(q: np.ndarray) -> np.ndarray:
    q = _as_vec4(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Hamilton product p ⊗ q (composition of rotations)."""
    p0, p1, p2, p3 = _as_vec4(p)
    q0, q1, q2, q3 = _as_vec4(q)
    return np.array(
        [
            p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3,
            p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2,
            p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1,
            p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0,
        ]
    )


def rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a 3-vector from body to inertial: v_i = R(q) v_b."""
    q = normalize(q)
    v = np.asarray(v, dtype=float).reshape(3)
    qv = np.array([0.0, v[0], v[1], v[2]])
    out = multiply(multiply(q, qv), conjugate(q))
    return out[1:]


def kinematics(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """Quaternion rate q̇ = 0.5 q ⊗ [0, ω]."""
    omega = np.asarray(omega, dtype=float).reshape(3)
    return 0.5 * multiply(q, np.array([0.0, omega[0], omega[1], omega[2]]))


def integrate(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """Exact exponential-map step: q ← q ⊗ [cos(θ/2), n̂ sin(θ/2)]."""
    omega = np.asarray(omega, dtype=float).reshape(3)
    angle = float(np.linalg.norm(omega) * dt)
    if angle < 1e-12:
        dq = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        axis = omega / np.linalg.norm(omega)
        half = 0.5 * angle
        dq = np.array([np.cos(half), *(axis * np.sin(half))])
    return normalize(multiply(q, dq))


def from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX intrinsic Euler angles (yaw-pitch-roll) to quaternion."""
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    return normalize(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ]
        )
    )


def to_euler(q: np.ndarray) -> np.ndarray:
    """Quaternion to ZYX Euler angles [roll, pitch, yaw] (radians).

    Pitch is clamped to avoid NaNs, but the mapping itself is singular
    near ±90° — that is the gimbal-lock issue this project avoids in control.
    """
    q0, q1, q2, q3 = normalize(q)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    roll = np.arctan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    yaw = np.arctan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
    return np.array([roll, pitch, yaw])


def rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Body-to-inertial rotation matrix R(q) ∈ SO(3)."""
    q0, q1, q2, q3 = normalize(q)
    return np.array(
        [
            [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
        ]
    )


def from_rotation_matrix(R: np.ndarray) -> np.ndarray:
    """Shepperd's method: rotation matrix to unit quaternion."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        q0 = 0.25 / s
        q1 = (R[2, 1] - R[1, 2]) * s
        q2 = (R[0, 2] - R[2, 0]) * s
        q3 = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q0 = (R[2, 1] - R[1, 2]) / s
        q1 = 0.25 * s
        q2 = (R[0, 1] + R[1, 0]) / s
        q3 = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q0 = (R[0, 2] - R[2, 0]) / s
        q1 = (R[0, 1] + R[1, 0]) / s
        q2 = 0.25 * s
        q3 = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q0 = (R[1, 0] - R[0, 1]) / s
        q1 = (R[0, 2] + R[2, 0]) / s
        q2 = (R[1, 2] + R[2, 1]) / s
        q3 = 0.25 * s
    return normalize(np.array([q0, q1, q2, q3]))


def error(q_desired: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """Attitude error quaternion q_e = q_desired ⊗ q_current⁻¹.

    Identity [1, 0, 0, 0] means the vehicle is already at the target.
    The shortest path is selected by flipping the sign if q_e0 < 0
    (q and −q represent the same rotation).
    """
    q_e = multiply(q_desired, conjugate(q_current))
    if q_e[0] < 0:
        q_e = -q_e
    return normalize(q_e)


def vector_error(q_desired: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """Vector part of the error quaternion — the PID error signal."""
    return error(q_desired, q_current)[1:]


def from_two_vectors_yaw(body_z: np.ndarray, yaw: float) -> np.ndarray:
    """Desired attitude whose body-z aligns with a thrust vector, at a given yaw."""
    b3 = np.asarray(body_z, dtype=float).reshape(3)
    n = np.linalg.norm(b3)
    if n < EPS:
        b3 = np.array([0.0, 0.0, 1.0])
    else:
        b3 = b3 / n
    c, s = np.cos(yaw), np.sin(yaw)
    b1_des = np.array([c, s, 0.0])
    b2 = np.cross(b3, b1_des)
    n2 = np.linalg.norm(b2)
    if n2 < 1e-6:
        b1_des = np.array([-s, c, 0.0])
        b2 = np.cross(b3, b1_des)
        n2 = np.linalg.norm(b2)
    b2 = b2 / n2
    b1 = np.cross(b2, b3)
    R = np.column_stack((b1, b2, b3))
    return from_rotation_matrix(R)


class Quaternion:
    """Thin object wrapper used in demos and notebooks."""

    def __init__(self, value: np.ndarray | None = None):
        self.q = normalize(value if value is not None else np.array([1.0, 0.0, 0.0, 0.0]))

    def __repr__(self) -> str:
        return f"Quaternion({self.q})"
