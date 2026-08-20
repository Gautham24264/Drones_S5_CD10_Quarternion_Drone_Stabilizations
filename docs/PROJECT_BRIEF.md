# Project brief

**Title.** Quaternion-Based Drone Attitude Stabilization Using IMU Sensor Fusion and PID Control

**Base paper.** A. Shevidi and H. A. Hashim, “Quaternion-based Adaptive Backstepping Fast Terminal Sliding Mode Control for Quadrotor UAVs with Finite Time Convergence,” arXiv:2407.01275, 2024. https://arxiv.org/abs/2407.01275

## Goal

Keep the paper’s singularity-free quaternion attitude pipeline, replace the advanced control laws with a PID that can be built on an MPU-6050 plus a gimbal or a small quadrotor, and demonstrate the whole loop in a 6-DOF simulation suitable for an oral presentation.

## What is kept from the paper

- Unit-quaternion attitude state \(q = [q_0, q_1, q_2, q_3]\)
- Quaternion kinematics \(\dot q = \tfrac12 q \otimes [0,\omega]\)
- Attitude error as a quaternion product
- Rigid-body quadrotor rotational dynamics
- Comparison against Euler-angle control

## What is simplified

| Paper | This project |
| --- | --- |
| Adaptive backstepping position control | Cascaded P/D on position → desired thrust vector |
| Adaptive fast terminal sliding mode attitude control | PID on \(q_{e,v}\) with gyro rate damping |
| Unknown time-varying parameter adaptation | Fixed gains + IMU noise / wind gusts |
| NED-style gravity in the paper’s equations | ENU (z up) for the 3D demo |

## Deliverables

1. Python simulation (IMU, Mahony, quaternion PID, 6-DOF quadrotor)
2. Comparison against Euler PID near gimbal lock
3. Presentation slides, speaker notes, 3D viewer, result figures and GIFs
4. Clear hardware mapping for a later MPU-6050 implementation
