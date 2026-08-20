import unittest

import numpy as np

from simulation.scenarios import gimbal_lock_logs, hover_mission_log, recovery_logs, step_tracking_log


def _tail_deg(log, seconds=0.5):
    a = log.as_arrays()
    mask = a["t"] >= a["t"][-1] - seconds
    return np.rad2deg(np.max(np.abs(a["euler"][mask]), axis=0))


class ClosedLoopTests(unittest.TestCase):
    def test_quaternion_recovery_settles(self):
        quat, _ = recovery_logs(75.0)
        peak = _tail_deg(quat)
        self.assertLess(float(np.max(peak)), 5.0)
        qn = np.linalg.norm(quat.as_arrays()["q"], axis=1)
        np.testing.assert_allclose(qn, 1.0, atol=1e-9)

    def test_gimbal_lock_euler_torque_spikes(self):
        quat, euler = gimbal_lock_logs()
        q_tau = float(np.max(np.abs(quat.as_arrays()["torque"])))
        e_tau = float(np.max(np.abs(euler.as_arrays()["torque"])))
        self.assertLess(q_tau, 12.0)
        self.assertGreater(e_tau, 40.0)
        self.assertLess(float(np.max(_tail_deg(quat))), 6.0)

    def test_step_tracking_follows_commands(self):
        log = step_tracking_log().as_arrays()
        # After the last step (t>5.8 s) stay near level.
        mask = log["t"] > 5.8
        err = np.rad2deg(np.max(np.abs(log["euler"][mask]), axis=0))
        self.assertLess(float(err[0]), 4.0)
        self.assertLess(float(err[1]), 4.0)

    def test_mission_tracks_figure_eight(self):
        log = hover_mission_log(duration=6.0).as_arrays()
        err = np.linalg.norm(log["position"] - log["p_des"], axis=1)
        late = err[log["t"] > 3.0]
        self.assertLess(float(np.mean(late)), 0.55)
        self.assertGreater(float(np.mean(log["position"][log["t"] > 3.0, 2])), 1.2)
        self.assertTrue(np.all(np.isfinite(log["q"])))
        qn = np.linalg.norm(log["q"], axis=1)
        np.testing.assert_allclose(qn, 1.0, atol=1e-8)


if __name__ == "__main__":
    unittest.main()
