"""Closed-loop simulation scenarios used for figures and the 3D demo."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from src.controllers import CascadedPositionController, EulerAttitudePID, QuaternionAttitudePID
from src.imu import IMU, IMUNoise
from src.mahony import MahonyFilter
from src.mixer import mixer_x
from src.quadrotor import Quadrotor, QuadrotorParams
from src import quaternion as Q


@dataclass
class SimLog:
    t: list = field(default_factory=list)
    q: list = field(default_factory=list)
    q_est: list = field(default_factory=list)
    q_des: list = field(default_factory=list)
    euler: list = field(default_factory=list)
    euler_est: list = field(default_factory=list)
    omega: list = field(default_factory=list)
    position: list = field(default_factory=list)
    p_des: list = field(default_factory=list)
    velocity: list = field(default_factory=list)
    torque: list = field(default_factory=list)
    thrust: list = field(default_factory=list)
    motors: list = field(default_factory=list)
    q_err_vec: list = field(default_factory=list)

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {
            "t": np.asarray(self.t),
            "q": np.asarray(self.q),
            "q_est": np.asarray(self.q_est),
            "q_des": np.asarray(self.q_des),
            "euler": np.asarray(self.euler),
            "euler_est": np.asarray(self.euler_est),
            "omega": np.asarray(self.omega),
            "position": np.asarray(self.position),
            "p_des": np.asarray(self.p_des),
            "velocity": np.asarray(self.velocity),
            "torque": np.asarray(self.torque),
            "thrust": np.asarray(self.thrust),
            "motors": np.asarray(self.motors),
            "q_err_vec": np.asarray(self.q_err_vec),
        }


def _record(
    log: SimLog,
    t: float,
    drone: Quadrotor,
    q_est,
    q_des,
    torque,
    thrust,
    p_des: np.ndarray | None = None,
) -> None:
    log.t.append(t)
    log.q.append(drone.q.copy())
    log.q_est.append(np.asarray(q_est, dtype=float).copy())
    log.q_des.append(np.asarray(q_des, dtype=float).copy())
    log.euler.append(Q.to_euler(drone.q))
    log.euler_est.append(Q.to_euler(q_est))
    log.omega.append(drone.omega.copy())
    log.position.append(drone.position.copy())
    log.p_des.append(np.asarray(p_des if p_des is not None else drone.position, dtype=float).copy())
    log.velocity.append(drone.velocity.copy())
    log.torque.append(np.asarray(torque, dtype=float).copy())
    log.thrust.append(float(thrust))
    log.motors.append(mixer_x(thrust, torque, drone.hover_thrust, drone.p.arm_length))
    log.q_err_vec.append(Q.vector_error(q_des, q_est))


def simulate_attitude(
    controller,
    q0: np.ndarray,
    q_des: np.ndarray | Callable[[float], np.ndarray],
    duration: float = 6.0,
    dt: float = 0.002,
    use_imu: bool = True,
    wind: Callable[[float], np.ndarray] | None = None,
    hold_altitude: bool = True,
    omega0: np.ndarray | None = None,
    attitude_only: bool = True,
) -> SimLog:
    """Closed-loop attitude test.

    Translation is frozen by default so the experiment isolates orientation
    (standard 3-DOF attitude-stabilization bench). IMU still sees gravity
    in the body frame plus sensor noise.
    """
    drone = Quadrotor(
        quaternion=q0,
        position=np.array([0.0, 0.0, 1.2]),
        omega=np.zeros(3) if omega0 is None else np.asarray(omega0, dtype=float),
    )
    imu = IMU(IMUNoise(seed=3))
    filt = MahonyFilter(kp=1.8, ki=0.04, q0=q0)
    log = SimLog()
    steps = int(duration / dt)
    for k in range(steps):
        t = k * dt
        q_target = q_des(t) if callable(q_des) else q_des
        if use_imu:
            accel_i = np.zeros(3) if attitude_only else drone.last_accel
            gyro, accel = imu.measure(drone.q, drone.omega, accel_i)
            q_hat = filt.update(gyro, accel, dt)
        else:
            q_hat = drone.q
        torque = controller.update(q_target, q_hat, drone.omega, dt)
        thrust = drone.hover_thrust
        if hold_altitude and not attitude_only:
            thrust = drone.hover_thrust + 6.0 * (1.2 - drone.position[2]) - 3.5 * drone.velocity[2]
        w = wind(t) if wind else None
        drone.step(thrust, torque, dt, wind=w, attitude_only=attitude_only)
        _record(log, t, drone, q_hat, q_target, torque, thrust, p_des=np.array([0.0, 0.0, 1.2]))
    return log


def recovery_logs(pitch0_deg: float = 75.0) -> tuple[SimLog, SimLog]:
    q0 = Q.from_euler(0.15, np.deg2rad(pitch0_deg), 0.4)
    q_des = Q.from_euler(0.0, 0.0, 0.0)
    quat = simulate_attitude(QuaternionAttitudePID(), q0, q_des, duration=5.0)
    euler = simulate_attitude(EulerAttitudePID(), q0, q_des, duration=5.0)
    return quat, euler


def gimbal_lock_logs() -> tuple[SimLog, SimLog]:
    """Start near 90° pitch so the Euler Jacobian is nearly singular."""
    q0 = Q.from_euler(0.20, np.deg2rad(89.4), -0.5)
    q_des = Q.from_euler(0.0, 0.0, 0.0)
    omega0 = np.array([0.0, 0.0, 1.8])
    quat = simulate_attitude(
        QuaternionAttitudePID(kp=5.0, kd=0.65), q0, q_des, duration=6.0, omega0=omega0
    )
    euler = simulate_attitude(
        EulerAttitudePID(kp=5.0, kd=0.65), q0, q_des, duration=6.0, omega0=omega0
    )
    return quat, euler


def step_tracking_log() -> SimLog:
    def q_des(t: float) -> np.ndarray:
        if t < 1.0:
            return Q.from_euler(0.0, 0.0, 0.0)
        if t < 3.0:
            return Q.from_euler(0.35, -0.25, 0.15)
        if t < 5.0:
            return Q.from_euler(-0.30, 0.35, -0.20)
        return Q.from_euler(0.0, 0.0, 0.0)

    return simulate_attitude(
        QuaternionAttitudePID(),
        Q.from_euler(0.1, -0.1, 0.05),
        q_des,
        duration=7.0,
    )


def fusion_log() -> SimLog:
    q0 = Q.from_euler(0.4, -0.3, 0.2)
    return simulate_attitude(
        QuaternionAttitudePID(),
        q0,
        Q.from_euler(0.0, 0.0, 0.0),
        duration=5.0,
        use_imu=True,
    )


def hover_mission_log(duration: float = 12.0, dt: float = 0.002) -> SimLog:
    drone = Quadrotor(
        quaternion=Q.from_euler(0.2, -0.15, 0.1),
        position=np.array([0.0, 0.0, 0.4]),
        params=QuadrotorParams(),
    )
    att = QuaternionAttitudePID(kp=5.5, ki=0.12, kd=0.7)
    pos = CascadedPositionController(mass=drone.p.mass, kp_pos=1.8, kd_pos=2.2)
    imu = IMU(IMUNoise(seed=11, gyro_std=0.008, accel_std=0.12))
    filt = MahonyFilter(kp=2.0, ki=0.05, q0=drone.q)
    log = SimLog()
    steps = int(duration / dt)
    for k in range(steps):
        t = k * dt
        p_des, v_des, a_des = _figure8(t)
        gyro, accel = imu.measure(drone.q, drone.omega, drone.last_accel)
        q_hat = filt.update(gyro, accel, dt)
        thrust, q_des = pos.update(p_des, drone.position, drone.velocity, v_des, a_des)
        torque = att.update(q_des, q_hat, drone.omega, dt)
        gust = np.array([0.35 * np.sin(0.7 * t), 0.2 * np.cos(0.5 * t), 0.0]) if t > 4.0 else np.zeros(3)
        drone.step(thrust, torque, dt, wind=gust, attitude_only=False)
        _record(log, t, drone, q_hat, q_des, torque, thrust, p_des=p_des)
    return log


def _figure8(t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Slow 3D figure-eight at 1.5 m, plus velocity and acceleration feedforward."""
    warmup = np.clip(t / 2.5, 0.0, 1.0)
    wx, wy = 0.45, 0.90
    ax_, ay_ = 1.4, 0.9
    x = ax_ * np.sin(wx * t) * warmup
    y = ay_ * np.sin(wy * t) * warmup
    z = 1.5
    vx = ax_ * wx * np.cos(wx * t) * warmup
    vy = ay_ * wy * np.cos(wy * t) * warmup
    ax = -ax_ * wx * wx * np.sin(wx * t) * warmup
    ay = -ay_ * wy * wy * np.sin(wy * t) * warmup
    return np.array([x, y, z]), np.array([vx, vy, 0.0]), np.array([ax, ay, 0.0])
