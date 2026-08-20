"""X-configuration motor mixing (for plots and hardware mapping).

    m_i = m_hover + u_roll a_i + u_pitch b_i + u_yaw c_i
"""

from __future__ import annotations

import numpy as np


def mixer_x(thrust: float, torque: np.ndarray, hover: float, arm: float = 0.2) -> np.ndarray:
    """Return four motor commands [front-right, rear-right, rear-left, front-left]."""
    tau = np.asarray(torque, dtype=float).reshape(3)
    roll, pitch, yaw = tau
    # Scale torques into a comparable command range.
    r, p, y = roll / max(arm, 1e-6), pitch / max(arm, 1e-6), yaw
    motors = np.array(
        [
            thrust / 4 + r + p - y,
            thrust / 4 - r + p + y,
            thrust / 4 - r - p - y,
            thrust / 4 + r - p + y,
        ]
    )
    return np.clip(motors, 0.0, hover * 2.5)
