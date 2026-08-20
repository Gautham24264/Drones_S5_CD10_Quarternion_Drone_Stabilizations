"""Generate every presentation figure, animation, and data file."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation import scenarios
from simulation.animate import animate_log
from simulation.figures import (
    plot_attitude_compare,
    plot_block_diagram,
    plot_error_and_control,
    plot_fusion,
    plot_metrics_table,
    plot_quaternion_states,
    plot_torque_compare,
    plot_trajectory,
    plot_xyz,
)


def _dump_json(log: dict, path: Path, stride: int = 8) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "t": log["t"][::stride].tolist(),
        "q": log["q"][::stride].tolist(),
        "position": log["position"][::stride].tolist(),
        "p_des": log["p_des"][::stride].tolist() if "p_des" in log else [],
        "motors": log["motors"][::stride].tolist(),
    }
    path.write_text(json.dumps(payload))


def _metrics(name: str, log: dict, extra: dict | None = None) -> dict:
    t = log["t"]
    tail = t >= t[-1] - 0.5
    eul = np.rad2deg(np.abs(log["euler"][tail]))
    out = {
        "name": name,
        "final_roll_deg": float(np.rad2deg(log["euler"][-1, 0])),
        "final_pitch_deg": float(np.rad2deg(log["euler"][-1, 1])),
        "final_yaw_deg": float(np.rad2deg(log["euler"][-1, 2])),
        "tail_max_abs_euler_deg": float(np.max(eul)),
        "max_abs_torque_nm": float(np.max(np.abs(log["torque"]))),
        "max_quat_norm_error": float(np.max(np.abs(np.linalg.norm(log["q"], axis=1) - 1.0))),
    }
    if extra:
        out.update(extra)
    return out


def main() -> None:
    results = ROOT / "results"
    figs = results / "figures"
    anims = results / "animations"
    data = results / "data"
    figs.mkdir(parents=True, exist_ok=True)
    data.mkdir(parents=True, exist_ok=True)
    anims.mkdir(parents=True, exist_ok=True)

    print("1/6  Block diagram")
    plot_block_diagram(figs / "block_diagram.png")

    print("2/6  Recovery + gimbal-lock comparison")
    rec_q, rec_e = scenarios.recovery_logs(75.0)
    rec_q_a, rec_e_a = rec_q.as_arrays(), rec_e.as_arrays()
    plot_attitude_compare(
        rec_q_a,
        rec_e_a,
        figs / "recovery_compare.png",
        "Large-angle recovery (75° initial pitch)",
    )
    plot_quaternion_states(rec_q_a, figs / "recovery_quaternion.png")
    plot_error_and_control(rec_q_a, figs / "recovery_control.png")
    plot_metrics_table(rec_q_a, rec_e_a, figs / "recovery_metrics.png")

    gl_q, gl_e = scenarios.gimbal_lock_logs()
    plot_attitude_compare(
        gl_q.as_arrays(),
        gl_e.as_arrays(),
        figs / "gimbal_lock_compare.png",
        "Near gimbal lock (89.4° pitch + yaw rate)",
    )
    plot_torque_compare(
        gl_q.as_arrays(),
        gl_e.as_arrays(),
        figs / "gimbal_lock_torque.png",
        "Commanded torque near gimbal lock (Euler inversion spikes)",
    )

    print("3/6  Step tracking and IMU fusion")
    from matplotlib import pyplot as plt
    from src import quaternion as Q

    step = scenarios.step_tracking_log().as_arrays()
    t, eul = step["t"], step["euler"]
    ed = np.array([Q.to_euler(q) for q in step["q_des"]])
    fig, axes = plt.subplots(3, 1, figsize=(9.5, 7.2), sharex=True)
    labels = ["Roll", "Pitch", "Yaw"]
    for i, ax in enumerate(axes):
        ax.plot(t, eul[:, i] * 180 / np.pi, color="#1f6feb", lw=2.0, label="Measured")
        ax.plot(t, ed[:, i] * 180 / np.pi, color="#9a6700", lw=1.4, ls="--", label="Desired")
        ax.set_ylabel(f"{labels[i]} (°)")
    axes[0].legend(loc="upper right")
    axes[0].set_title("Quaternion PID — commanded attitude steps")
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(figs / "step_tracking.png", dpi=160, bbox_inches="tight")
    plt.close(fig)

    fusion = scenarios.fusion_log().as_arrays()
    plot_fusion(fusion, figs / "sensor_fusion.png")

    print("4/6  6-DOF hover / figure-eight mission")
    mission = scenarios.hover_mission_log().as_arrays()
    plot_trajectory(mission, figs / "trajectory3d.png")
    plot_xyz(mission, figs / "position_time.png")
    plot_error_and_control(mission, figs / "mission_control.png")
    np.savez(data / "mission.npz", **mission)
    _dump_json(mission, data / "mission.json", stride=10)
    _dump_json(rec_q_a, data / "recovery.json", stride=8)

    pos_err = np.linalg.norm(mission["position"] - mission["p_des"], axis=1)
    metrics = [
        _metrics("recovery_quaternion", rec_q_a),
        _metrics("recovery_euler", rec_e_a),
        _metrics("gimbal_quaternion", gl_q.as_arrays()),
        _metrics("gimbal_euler", gl_e.as_arrays()),
        _metrics(
            "mission",
            mission,
            extra={
                "mean_position_error_m_after_3s": float(np.mean(pos_err[mission["t"] > 3.0])),
                "mean_altitude_m_after_3s": float(np.mean(mission["position"][mission["t"] > 3.0, 2])),
            },
        ),
    ]
    (data / "metrics.json").write_text(json.dumps(metrics, indent=2))

    print("5/6  GIF animations (this takes a minute)")
    animate_log(rec_q_a, anims / "recovery.gif", stride=25)
    animate_log(mission, anims / "mission.gif", stride=28)

    print("6/6  Done")
    print(f"Figures    → {figs}")
    print(f"Animations → {anims}")
    print(f"Metrics    → {data / 'metrics.json'}")
    print("Open presentation/slides.html after this script finishes.")


if __name__ == "__main__":
    main()
