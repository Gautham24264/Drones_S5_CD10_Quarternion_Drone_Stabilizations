# Speaker notes (≈12–15 minutes)

Use **← →** or click left/right on `presentation/slides.html`. Keep the 3D viewer or GIFs as a backup if the live matplotlib window is slow.

## Slide 1 — Title
One sentence: *We took a 2024 quaternion sliding-mode paper and turned the geometry into something you can actually simulate and tune: IMU fusion plus PID.*

## Slide 2 — Problem
Show with your hands: pitch up toward 90°. Roll and yaw start describing the same motion. That is gimbal lock. A flip recovery is exactly when you do not want the controller to misbehave.

## Slide 3 — Paper vs project
Be explicit about what you did **not** implement (adaptive backstepping, fast terminal SMC, Lyapunov proofs) and what you **did** keep (unit quaternion, error quaternion, closed-loop quadrotor). Examiners like that honesty.

## Slides 4–7 — Method
Walk the pipeline slowly. Pause on \(q_e = q_d \otimes q^{-1}\). When \(q_e = [1,0,0,0]\) you are done. The PID sees only the vector part.

Mention ENU (z up) so nobody is confused by the paper’s NED-style gravity sign.

## Slides 8–9 — Results
Lead with the 75° recovery: both controllers work, quaternion is cleaner.
Then 88°: this is the money slide. Euler representation is singular; quaternion is not.

If asked about fairness: **same PID gains, same plant, same IMU.** The only change is the error signal.

## Slides 10–12 — Tracking and 6-DOF
Step commands show the inner loop. The figure-eight is the “it actually flies” slide. Wind gusts after t = 4 s are in the mission scenario.

## Slide 13 — Demo
Prefer the GIF if you are on a projector with no Python. Prefer `live_demo` or `viewer.html` if you can.

## Slide 14 — Hardware
MPU-6050, 200 Hz+, tune P then D then I. Gimbal first, flying frame later.

## Slide 15 — Close
Three claims only: singularity-free error, fusion is good enough, PID is enough to demonstrate the paper’s idea.

## Likely questions
- **Why not implement SMC?** Timeline and chattering on real actuators. Geometry is the transferable idea.
- **Yaw drift?** Accelerometer cannot observe yaw; a magnetometer or optical flow would pin it. We still stabilize roll/pitch.
- **Why Mahony not Madgwick?** Same correction idea; Mahony is a few lines and easy to explain.
- **Stability proof?** Not claimed. The paper proves AFTSMC. We show empirical recovery and comparison.
