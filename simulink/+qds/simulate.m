function log = simulate(scenario, opts)
% Unified closed-loop simulator (matches Python simulation/scenarios.py).
arguments
    scenario (1, :) char {mustBeMember(scenario, {'recovery', 'recovery_euler', 'gimbal', 'gimbal_euler', 'step', 'fusion', 'mission', 'helix'})}
    opts.Duration (1, 1) double = 12
    opts.Dt (1, 1) double = 0.002
    opts.ImuSeed (1, 1) double = 11
    opts.UseImu (1, 1) logical = true
    opts.UseMag (1, 1) logical = true
    opts.UseActuator (1, 1) logical = true
    opts.LiveCallback = function_handle.empty
    opts.Stride (1, 1) double = 8
end

dt = opts.Dt;
N = floor(opts.Duration / dt);
log = empty_log(N);

use_euler = contains(scenario, 'euler');
attitude_only = ~ismember(scenario, {'mission', 'helix'});
path_fn = [];
omega0 = zeros(3, 1);

switch scenario
    case {'recovery', 'recovery_euler'}
        opts.Duration = min(opts.Duration, 5);
        N = floor(opts.Duration / dt);
        log = empty_log(N);
        q0 = qds.euler_to_quat(0.15, deg2rad(75), 0.4);
        state = [0; 0; 1.2; zeros(3, 1); q0(:); omega0];
        q_des = qds.euler_to_quat(0, 0, 0);
        mh = qds.mahony_init(q0, 1.8, 0.04, 0.4);
        imu = qds.imu_init(opts.ImuSeed, 0.01, 0.15, 0.02);
        pid_i = zeros(3, 1);
        kp = 4.5; ki = 0.15; kd = 0.55;
        act = qds.actuator_init(9.81);
    case {'gimbal', 'gimbal_euler'}
        opts.Duration = min(opts.Duration, 6);
        N = floor(opts.Duration / dt);
        log = empty_log(N);
        q0 = qds.euler_to_quat(0.20, deg2rad(89.4), -0.5);
        omega0 = [0; 0; 1.8];
        state = [0; 0; 1.2; zeros(3, 1); q0(:); omega0];
        q_des = qds.euler_to_quat(0, 0, 0);
        mh = qds.mahony_init(q0, 1.8, 0.04, 0.4);
        imu = qds.imu_init(opts.ImuSeed, 0.01, 0.15, 0.02);
        pid_i = zeros(3, 1);
        kp = 5.0; ki = 0.15; kd = 0.65;
        act = qds.actuator_init(9.81);
    case 'step'
        opts.Duration = min(opts.Duration, 7);
        N = floor(opts.Duration / dt);
        log = empty_log(N);
        q0 = qds.euler_to_quat(0.1, -0.1, 0.05);
        state = [0; 0; 1.2; zeros(3, 1); q0(:); zeros(3, 1)];
        q_des = qds.euler_to_quat(0, 0, 0);
        mh = qds.mahony_init(q0, 1.8, 0.04, 0.4);
        imu = qds.imu_init(opts.ImuSeed, 0.01, 0.15, 0.02);
        pid_i = zeros(3, 1);
        kp = 4.5; ki = 0.15; kd = 0.55;
        act = qds.actuator_init(9.81);
    case 'fusion'
        opts.Duration = min(opts.Duration, 5);
        N = floor(opts.Duration / dt);
        log = empty_log(N);
        q0 = qds.euler_to_quat(0.4, -0.3, 0.2);
        state = [0; 0; 1.2; zeros(3, 1); q0(:); zeros(3, 1)];
        q_des = qds.euler_to_quat(0, 0, 0);
        mh = qds.mahony_init(q0, 1.8, 0.04, 0.4);
        imu = qds.imu_init(opts.ImuSeed, 0.01, 0.15, 0.02);
        pid_i = zeros(3, 1);
        kp = 4.5; ki = 0.15; kd = 0.55;
        act = qds.actuator_init(9.81);
    case 'mission'
        q0 = qds.euler_to_quat(0.2, -0.15, 0.1);
        state = [0; 0; 0.4; zeros(3, 1); q0(:); zeros(3, 1)];
        mh = qds.mahony_init(q0, 2.0, 0.05, 0.4);
        imu = qds.imu_init(opts.ImuSeed, 0.008, 0.12, 0.02);
        pid_i = zeros(3, 1);
        kp = 5.5; ki = 0.12; kd = 0.7;
        act = qds.actuator_init(9.81);
        path_fn = @qds.figure8_ref;
        q_des = qds.euler_to_quat(0, 0, 0);
    case 'helix'
        q0 = qds.euler_to_quat(0.2, -0.15, 0.1);
        state = [0; 0; 0.4; zeros(3, 1); q0(:); zeros(3, 1)];
        mh = qds.mahony_init(q0, 2.0, 0.05, 0.4);
        imu = qds.imu_init(opts.ImuSeed, 0.008, 0.12, 0.02);
        pid_i = zeros(3, 1);
        kp = 5.5; ki = 0.12; kd = 0.7;
        act = qds.actuator_init(9.81);
        path_fn = @qds.helix_ref;
        q_des = qds.euler_to_quat(0, 0, 0);
end

last_accel = zeros(3, 1);

for k = 1:N
    t = (k - 1) * dt;
    if attitude_only
        q_target = step_q_des(scenario, t, q_des);
        p_des = [0; 0; 1.2];
        thrust_cmd = 9.81;
    else
        [p_des, v_des, a_ff] = path_fn(t);
        [thrust_cmd, q_target] = qds.cascade_position(p_des, state(1:3), state(4:6), v_des, a_ff, 1.0, 9.81, 1.8, 2.2);
    end

    if opts.UseImu
        accel_i = zeros(3, 1);
        if ~attitude_only
            accel_i = last_accel;
        end
        [gyro, accel, mag] = qds.imu_measure(imu, state(7:10), state(11:13), accel_i, false);
        if opts.UseMag
            [mh, q_hat] = qds.mahony_update(mh, gyro, accel, dt, mag);
        else
            [mh, q_hat] = qds.mahony_update(mh, gyro, accel, dt, []);
        end
    else
        q_hat = state(7:10);
    end

    if use_euler
        [tau_cmd, pid_i] = qds.pid_attitude_euler(q_target, q_hat, state(11:13), dt, kp, ki, kd, pid_i);
    else
        [tau_cmd, pid_i] = qds.pid_attitude(q_target, q_hat, state(11:13), dt, kp, ki, kd, pid_i);
    end

    if opts.UseActuator
        [act, thrust, tau, ~] = qds.actuator_apply(act, thrust_cmd, tau_cmd, dt, []);
    else
        thrust = thrust_cmd;
        tau = tau_cmd;
    end

    wind = zeros(3, 1);
    if ~attitude_only && t > 4
        wind = [0.35 * sin(0.7 * t); 0.2 * cos(0.5 * t); 0];
    end

    [state, last_accel] = qds.quadrotor_step(state, thrust, tau, dt, wind, attitude_only);

    log.t(k) = t;
    log.p(k, :) = state(1:3).';
    log.p_des(k, :) = p_des(:).';
    log.q(k, :) = state(7:10).';
    log.q_hat(k, :) = q_hat(:).';
    log.q_des(k, :) = q_target(:).';
    log.euler(k, :) = qds.quat_to_euler(state(7:10)).';
    log.euler_est(k, :) = qds.quat_to_euler(q_hat).';
    log.torque_cmd(k, :) = tau_cmd(:).';
    log.torque(k, :) = tau(:).';
    log.thrust_cmd(k) = thrust_cmd;

    if ~isempty(opts.LiveCallback) && mod(k - 1, opts.Stride) == 0
        stop = feval(opts.LiveCallback, t, state(1:3), state(7:10));
        if islogical(stop) && stop
            log = trim_log(log, k);
            break;
        end
    end
end
end

function log = trim_log(log, k)
fn = fieldnames(log);
for i = 1:numel(fn)
    log.(fn{i}) = log.(fn{i})(1:k, :);
end
end

function log = empty_log(N)
log.t = zeros(N, 1);
log.p = zeros(N, 3);
log.p_des = zeros(N, 3);
log.q = zeros(N, 4);
log.q_hat = zeros(N, 4);
log.q_des = zeros(N, 4);
log.euler = zeros(N, 3);
log.euler_est = zeros(N, 3);
log.torque_cmd = zeros(N, 3);
log.torque = zeros(N, 3);
log.thrust_cmd = zeros(N, 1);
end

function qd = step_q_des(scenario, t, q_level)
if ~strcmp(scenario, 'step')
    qd = q_level;
    return
end
if t < 1.0
    qd = qds.euler_to_quat(0, 0, 0);
elseif t < 3.0
    qd = qds.euler_to_quat(0.35, -0.25, 0.15);
elseif t < 5.0
    qd = qds.euler_to_quat(-0.30, 0.35, -0.20);
else
    qd = qds.euler_to_quat(0, 0, 0);
end
end
