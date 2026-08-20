"""3D quadrotor animation (matplotlib) for the live demo and GIF export."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib import animation
from matplotlib import pyplot as plt

from src import quaternion as Q


def _body_points(q: np.ndarray, p: np.ndarray, arm: float = 0.28) -> dict[str, np.ndarray]:
    R = Q.rotation_matrix(q)
    origin = np.asarray(p, dtype=float)
    x_b = R @ np.array([arm, 0.0, 0.0])
    y_b = R @ np.array([0.0, arm, 0.0])
    z_b = R @ np.array([0.0, 0.0, 0.18])
    rotors = [
        origin + R @ np.array([arm, arm, 0.0]) * 0.7,
        origin + R @ np.array([-arm, arm, 0.0]) * 0.7,
        origin + R @ np.array([-arm, -arm, 0.0]) * 0.7,
        origin + R @ np.array([arm, -arm, 0.0]) * 0.7,
    ]
    return {
        "origin": origin,
        "up": origin + z_b,
        "rotors": rotors,
        "arm_a": (origin - x_b, origin + x_b),
        "arm_b": (origin - y_b, origin + y_b),
    }


def _fit_limits(p: np.ndarray) -> tuple[np.ndarray, float]:
    lo = p.min(axis=0)
    hi = p.max(axis=0)
    center = 0.5 * (lo + hi)
    radius = float(np.max(hi - lo) * 0.5) + 0.85
    radius = max(radius, 1.4)
    return center, radius


def animate_log(log: dict, path: Path | None = None, live: bool = False, stride: int = 12) -> None:
    t = log["t"][::stride]
    q = log["q"][::stride]
    p = log["position"][::stride]
    fig = plt.figure(figsize=(8.5, 7.0))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Quaternion attitude stabilization — quadrotor")
    center, radius = _fit_limits(p)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0.0, center[2] - radius), center[2] + radius)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass

    (arm_a,) = ax.plot([], [], [], color="#d1242f", lw=3.5)
    (arm_b,) = ax.plot([], [], [], color="#0969da", lw=3.5)
    (up,) = ax.plot([], [], [], color="#1a7f37", lw=2.0)
    rotors = [ax.plot([], [], [], "o", color="#24292f", ms=7)[0] for _ in range(4)]
    (trail,) = ax.plot([], [], [], color="#8c959f", lw=1.0, alpha=0.7)
    time_txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

    def update(i: int):
        body = _body_points(q[i], p[i])
        a0, a1 = body["arm_a"]
        b0, b1 = body["arm_b"]
        arm_a.set_data_3d([a0[0], a1[0]], [a0[1], a1[1]], [a0[2], a1[2]])
        arm_b.set_data_3d([b0[0], b1[0]], [b0[1], b1[1]], [b0[2], b1[2]])
        o, u = body["origin"], body["up"]
        up.set_data_3d([o[0], u[0]], [o[1], u[1]], [o[2], u[2]])
        for artist, r in zip(rotors, body["rotors"]):
            artist.set_data_3d([r[0]], [r[1]], [r[2]])
        trail.set_data_3d(p[: i + 1, 0], p[: i + 1, 1], p[: i + 1, 2])
        time_txt.set_text(f"t = {t[i]:.2f} s")
        return [arm_a, arm_b, up, trail, time_txt, *rotors]

    update(0)
    n = len(t)
    anim = animation.FuncAnimation(
        fig, update, frames=n, interval=30, blit=False, init_func=lambda: update(0)
    )
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        anim.save(path, writer="pillow", fps=24)
        plt.close(fig)
        return
    if live:
        plt.show()
    else:
        plt.close(fig)
