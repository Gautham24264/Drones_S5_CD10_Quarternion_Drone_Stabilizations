import unittest

import numpy as np

from src import quaternion as Q
from src.mahony import MahonyFilter
from src.quadrotor import Quadrotor


class QuaternionTests(unittest.TestCase):
    def test_identity_error(self):
        q = Q.from_euler(0.2, -0.1, 0.4)
        err = Q.error(q, q)
        self.assertAlmostEqual(err[0], 1.0, places=9)
        self.assertLess(np.linalg.norm(err[1:]), 1e-9)

    def test_roundtrip_euler_small(self):
        e = np.array([0.3, -0.2, 0.5])
        q = Q.from_euler(*e)
        e2 = Q.to_euler(q)
        np.testing.assert_allclose(e, e2, atol=1e-9)

    def test_rotation_matrix_orthonormal(self):
        q = Q.from_euler(0.7, -0.4, 1.1)
        R = Q.rotation_matrix(q)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=9)
        q2 = Q.from_rotation_matrix(R)
        self.assertTrue(np.allclose(q2, q, atol=1e-8) or np.allclose(q2, -q, atol=1e-8))

    def test_integrate_preserves_norm(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        omega = np.array([0.3, -0.2, 0.8])
        for _ in range(200):
            q = Q.integrate(q, omega, 0.01)
        self.assertAlmostEqual(np.linalg.norm(q), 1.0, places=9)

    def test_exponential_map_90deg(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        q = Q.integrate(q, np.array([0.0, np.pi / 2, 0.0]), 1.0)
        eul = Q.to_euler(q)
        self.assertAlmostEqual(eul[1], np.pi / 2, places=6)

    def test_rotate_vector_matches_matrix(self):
        q = Q.from_euler(0.4, -0.25, 0.7)
        v = np.array([1.0, -2.0, 0.5])
        np.testing.assert_allclose(Q.rotate_vector(q, v), Q.rotation_matrix(q) @ v, atol=1e-9)


class MahonyTests(unittest.TestCase):
    def test_converges_roll_pitch_from_wrong_initial(self):
        true_q = Q.from_euler(0.35, -0.25, 0.0)
        filt = MahonyFilter(kp=3.0, ki=0.08, q0=np.array([1.0, 0.0, 0.0, 0.0]))
        accel = Q.rotation_matrix(true_q).T @ np.array([0.0, 0.0, 9.81])
        gyro = np.zeros(3)
        for _ in range(4000):
            filt.update(gyro, accel, 0.002)
        est = Q.to_euler(filt.q)
        true = Q.to_euler(true_q)
        self.assertLess(abs(est[0] - true[0]), np.deg2rad(2.0))
        self.assertLess(abs(est[1] - true[1]), np.deg2rad(2.0))


class PlantTests(unittest.TestCase):
    def test_hover_stays_level_with_hover_thrust(self):
        drone = Quadrotor()
        z0 = drone.position[2]
        for _ in range(500):
            drone.step(drone.hover_thrust, np.zeros(3), 0.002)
        self.assertLess(abs(drone.position[2] - z0), 0.02)
        self.assertLess(np.linalg.norm(Q.to_euler(drone.q)), np.deg2rad(1.0))

    def test_attitude_only_does_not_translate(self):
        drone = Quadrotor(quaternion=Q.from_euler(0.4, 0.6, 0.1), position=np.array([1.0, 2.0, 3.0]))
        p0 = drone.position.copy()
        for _ in range(200):
            drone.step(drone.hover_thrust, np.array([0.1, -0.1, 0.0]), 0.002, attitude_only=True)
        np.testing.assert_allclose(drone.position, p0)


if __name__ == "__main__":
    unittest.main()
