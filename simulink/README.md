# MATLAB simulation (primary demo)

All closed-loop physics run here at **500 Hz** (`dt = 0.002` s), matching the Python reference implementation:

- ENU rigid-body plant (`+qds/quadrotor_step.m`)
- IMU (gyro, accel, mag noise + bias) → Mahony fusion
- Cascaded position control → quaternion PID attitude
- Motor mixer with ESC lag, saturation, and battery sag

## Quick start

Open MATLAB (desktop, with graphics for live view):

```matlab
cd('/Users/bettim/Projects/quaternion-drone-stabilization/simulink')
setup

live_demo('mission')    % real-time figure-eight (~12 s)
live_demo('recovery')   % 75° pitch recovery (~5 s)
live_demo('helix')      % climbing helix (~14 s)
```

Regenerate **all** presentation figures and metrics:

```matlab
generate_results   % → results/figures/, results/data/, results/figures_matlab/
run_scenarios      % → results/data/matlab_logs/
```

Batch logs and offline figures (legacy aliases):

```matlab
run_scenarios      % → results/data/matlab_logs/*.mat
generate_results   % → results/figures_matlab/*.png
```

## Core API

```matlab
log = qds.simulate('mission', Duration=12, ImuSeed=11, UseActuator=true);
```

Scenarios: `'recovery'` | `'mission'` | `'helix'`.

Log fields: `t`, `p`, `p_des`, `q`, `q_hat`, `euler`, `thrust_cmd`, `torque_cmd`.

## Simulink model (optional)

`build_quadrotor_model` creates `quadrotor_multibody_plant.slx` (solver **ode4**, step **0.002**). Replace the placeholder plant with **Simscape Multibody** when licensed (`license('test','Simscape_Multibody')`).

The **authoritative demo** is `qds.simulate` / `live_demo`, not the skeleton `.slx`.

## Python

Python under `simulation/` remains for CI and batch figure export; **oral demos and reports should use MATLAB** above.
