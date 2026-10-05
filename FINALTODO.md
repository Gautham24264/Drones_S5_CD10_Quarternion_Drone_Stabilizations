Quaternion Drone Stabilization — Final Review TODO
Upgrades
-	Add a simplified version of the paper's backstepping / sliding-mode controller and compare it against the current PID.
-	Test mass/inertia uncertainty, sensor dropout, and single-rotor failure, not just wind gusts.
-	Add motor/ESC lag, thrust saturation, and battery voltage sag to the mixer stage.
-	Run each scenario over multiple randomized trials; report mean ± std instead of one run.
-	Add a simulated magnetometer so yaw becomes observable.
-	Add at least one or two more reference path (e.g. a helix) beyond the figure-eight.
-	Rebuild the pipeline in Simulink with a Simscape Multibody plant.
-	Update the README/report with new results and a limitations section for Q&A.
Simulink Build Steps
-	Set solver to ode4, fixed step 0.002 s to match the Python sample time.
-	Port imu.py and mahony.py into IMU and filter subsystems.
-	Port quaternion.py and controllers.py into a quaternion-error PID subsystem.
-	Port mixer.py as a gain block; replace quadrotor.py with a Simscape Multibody 6-DOF body (mass 1.0 kg, inertia diag(0.012, 0.012, 0.021)).
-	Sequence scenarios (recovery, gimbal-lock, mission) and log signals to compare against the existing Python plots.
