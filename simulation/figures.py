"""Matplotlib figures for the presentation."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "figure.facecolor": "white",
        "axes.facecolor": "#fbfbfd",
        "axes.titleweight": "bold",
    }
)

DEG = 180.0 / np.pi


def _save(fig: plt.Figure, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def plot_attitude_compare(quat, euler, path: Path, title: str) -> None:
    t = quat["t"]
    fig, axes = plt.subplots(3, 1, figsize=(9.5, 7.2), sharex=True)
    labels = ["Roll", "Pitch", "Yaw"]
    for i, ax in enumerate(axes):
        ax.plot(t, quat["euler"][:, i] * DEG, color="#1f6feb", lw=2.0, label="Quaternion PID")
        ax.plot(euler["t"], euler["euler"][:, i] * DEG, color="#d1242f", lw=1.6, ls="--", label="Euler PID")
        ax.axhline(0.0, color="#6e7781", lw=0.8, ls=":")
        ax.set_ylabel(f"{labels[i]} (°)")
        ax.set_ylim(-120, 120)
    axes[0].legend(loc="upper right", framealpha=0.95)
    axes[0].set_title(title)
    axes[-1].set_xlabel("Time (s)")
    _save(fig, path)


def plot_quaternion_states(log, path: Path) -> None:
    t, q, qd = log["t"], log["q"], log["q_des"]
    fig, ax = plt.subplots(figsize=(9.5, 4.4))
    names = [r"$q_0$", r"$q_1$", r"$q_2$", r"$q_3$"]
    colors = ["#24292f", "#1f6feb", "#1a7f37", "#bf3989"]
    for i in range(4):
        ax.plot(t, q[:, i], color=colors[i], lw=2.0, label=names[i])
        ax.plot(t, qd[:, i], color=colors[i], lw=1.1, ls="--", alpha=0.7)
    ax.set_title("Attitude quaternion (solid) vs desired (dashed)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Quaternion component")
    ax.legend(ncol=4, loc="lower right")
    _save(fig, path)


def plot_error_and_control(log, path: Path) -> None:
    t = log["t"]
    fig, axes = plt.subplots(2, 1, figsize=(9.5, 6.4), sharex=True)
    axes[0].plot(t, log["q_err_vec"][:, 0], label=r"$q_{e,x}$")
    axes[0].plot(t, log["q_err_vec"][:, 1], label=r"$q_{e,y}$")
    axes[0].plot(t, log["q_err_vec"][:, 2], label=r"$q_{e,z}$")
    axes[0].set_ylabel("Quaternion vector error")
    axes[0].set_title("Error signal fed to the PID loop")
    axes[0].legend(ncol=3)
    axes[1].plot(t, log["torque"][:, 0], label=r"$\tau_x$")
    axes[1].plot(t, log["torque"][:, 1], label=r"$\tau_y$")
    axes[1].plot(t, log["torque"][:, 2], label=r"$\tau_z$")
    axes[1].set_ylabel("Body torque (N·m)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Control effort")
    axes[1].legend(ncol=3)
    _save(fig, path)


def plot_fusion(log, path: Path) -> None:
    t = log["t"]
    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    for i, name in enumerate(["Roll", "Pitch", "Yaw"]):
        ax.plot(t, log["euler"][:, i] * DEG, lw=2.0, label=f"True {name.lower()}")
        ax.plot(t, log["euler_est"][:, i] * DEG, lw=1.3, ls="--", label=f"Mahony {name.lower()}")
    ax.set_title("IMU sensor fusion (Mahony) vs true attitude")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (°)")
    ax.legend(ncol=3, fontsize=8)
    _save(fig, path)


def plot_trajectory(log, path: Path) -> None:
    p = log["position"]
    fig = plt.figure(figsize=(8.6, 6.4))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color="#1f6feb", lw=2.0, label="Flown")
    if "p_des" in log and len(log["p_des"]):
        pd = log["p_des"]
        ax.plot(pd[:, 0], pd[:, 1], pd[:, 2], color="#9a6700", lw=1.2, ls="--", label="Desired")
    ax.scatter(p[0, 0], p[0, 1], p[0, 2], color="#1a7f37", s=50, label="Start")
    ax.scatter(p[-1, 0], p[-1, 1], p[-1, 2], color="#d1242f", s=50, label="End")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("Closed-loop quadrotor trajectory")
    ax.legend()
    ax.set_box_aspect((1, 1, 0.7))
    _save(fig, path)


def plot_xyz(log, path: Path) -> None:
    t, p = log["t"], log["position"]
    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    colors = ["#1f6feb", "#1a7f37", "#bf3989"]
    names = ["x", "y", "z"]
    for i in range(3):
        ax.plot(t, p[:, i], color=colors[i], lw=2.0, label=names[i])
        if "p_des" in log and len(log["p_des"]):
            ax.plot(t, log["p_des"][:, i], color=colors[i], lw=1.1, ls="--", alpha=0.75)
    ax.set_title("Position vs time (solid = flown, dashed = desired)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.legend(ncol=3)
    _save(fig, path)


def plot_block_diagram(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(11.2, 3.6))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 4)
    ax.axis("off")
    ax.set_title("Signal flow — IMU fusion, quaternion error, PID, quadrotor", pad=12)

    boxes = [
        (0.2, 1.3, "MPU-6050\nIMU"),
        (2.4, 1.3, "Mahony\nfusion"),
        (4.6, 1.3, "Quaternion\nerror"),
        (6.8, 1.3, "PID\ncontroller"),
        (9.0, 1.3, "Motor mix\n+ quadrotor"),
    ]
    for x, y, text in boxes:
        ax.add_patch(
            FancyBboxPatch(
                (x, y),
                1.9,
                1.5,
                boxstyle="round,pad=0.04,rounding_size=0.15",
                facecolor="#ddf4ff",
                edgecolor="#0969da",
                linewidth=1.6,
            )
        )
        ax.text(x + 0.95, y + 0.75, text, ha="center", va="center", fontsize=10, color="#0a3069")

    ax.add_patch(
        FancyBboxPatch(
            (4.6, 3.05),
            1.9,
            0.7,
            boxstyle="round,pad=0.03,rounding_size=0.12",
            facecolor="#fff8c5",
            edgecolor="#9a6700",
            linewidth=1.4,
        )
    )
    ax.text(5.55, 3.4, r"$q_{desired}$", ha="center", va="center", fontsize=10)

    for x0 in (2.1, 4.3, 6.5, 8.7):
        ax.annotate(
            "",
            xy=(x0 + 0.3, 2.05),
            xytext=(x0, 2.05),
            arrowprops=dict(arrowstyle="-|>", color="#24292f", lw=1.6),
        )

    ax.annotate(
        "",
        xy=(5.55, 2.8),
        xytext=(5.55, 3.05),
        arrowprops=dict(arrowstyle="-|>", color="#9a6700", lw=1.4),
    )
    ax.annotate(
        "",
        xy=(1.15, 1.3),
        xytext=(9.95, 0.55),
        arrowprops=dict(arrowstyle="-|>", color="#1a7f37", lw=1.5, connectionstyle="arc3,rad=0.18"),
    )
    ax.text(5.5, 0.28, "physical orientation feeds back through the IMU", ha="center", color="#1a7f37", fontsize=9)
    _save(fig, path)


def plot_torque_compare(quat, euler, path: Path, title: str) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(9.5, 7.2), sharex=True)
    labels = [r"$\tau_x$", r"$\tau_y$", r"$\tau_z$"]
    peak = float(np.max(np.abs(euler["torque"])))
    for i, ax in enumerate(axes):
        ax.plot(quat["t"], quat["torque"][:, i], color="#1f6feb", lw=2.0, label="Quaternion PID")
        ax.plot(euler["t"], euler["torque"][:, i], color="#d1242f", lw=1.4, ls="--", label="Euler PID")
        ax.set_ylabel(f"{labels[i]} (N·m)")
        ax.set_ylim(-12, 12)
    axes[0].legend(loc="upper right")
    axes[0].set_title(title)
    axes[0].text(
        0.02,
        0.08,
        f"Euler command peaks at {peak:.0f} N·m (off-scale); plant saturates at 2 N·m",
        transform=axes[0].transAxes,
        color="#d1242f",
        fontsize=9,
    )
    axes[-1].set_xlabel("Time (s)")
    _save(fig, path)


def plot_metrics_table(quat, euler, path: Path) -> None:
    def rms(log, t_start=1.5):
        mask = log["t"] >= t_start
        eul = log["euler"][mask]
        return np.sqrt(np.mean(eul**2, axis=0)) * DEG

    rq, re = rms(quat), rms(euler)
    fig, ax = plt.subplots(figsize=(8.8, 3.4))
    ax.axis("off")
    ax.set_title("Steady-state RMS attitude error after 1.5 s (°)")
    table = ax.table(
        cellText=[
            ["Quaternion PID", f"{rq[0]:.2f}", f"{rq[1]:.2f}", f"{rq[2]:.2f}"],
            ["Euler PID", f"{re[0]:.2f}", f"{re[1]:.2f}", f"{re[2]:.2f}"],
        ],
        colLabels=["Controller", "Roll", "Pitch", "Yaw"],
        loc="center",
        cellLoc="center",
    )
    table.scale(1.2, 2.0)
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    _save(fig, path)
