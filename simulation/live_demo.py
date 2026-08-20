"""Live 3D matplotlib demo of quaternion attitude recovery."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.animate import animate_log
from simulation.scenarios import hover_mission_log, recovery_logs


def main() -> None:
    print("Simulating recovery, then opening the 3D viewer…")
    print("Close the first window to see the figure-eight mission.")
    rec, _ = recovery_logs(80.0)
    animate_log(rec.as_arrays(), live=True, stride=10)
    print("Simulating 6-DOF figure-eight…")
    mission = hover_mission_log(duration=10.0)
    animate_log(mission.as_arrays(), live=True, stride=12)


if __name__ == "__main__":
    main()
