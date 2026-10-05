function [act, thrust, torque, motors] = actuator_apply(act, thrust_cmd, torque_cmd, dt, fault_idx)
motors_cmd = qds.mixer_x(thrust_cmd, torque_cmd(:), act.hover, act.arm);
if nargin >= 5 && ~isempty(fault_idx) && fault_idx >= 0 && fault_idx <= 3
    motors_cmd(fault_idx + 1) = 0;
    [thrust_cmd, torque_cmd] = qds.motors_to_wrench(motors_cmd, act.hover, act.arm);
    motors_cmd = qds.mixer_x(thrust_cmd, torque_cmd(:), act.hover, act.arm);
    motors_cmd(fault_idx + 1) = 0;
end
alpha = dt / (act.tau_esc + dt);
act.motors = (1 - alpha) * act.motors + alpha * motors_cmd(:);
v = act.v0 - act.sag_k * sum(act.motors);
scale = max(act.v_min / act.v0, min(1.0, v / act.v0));
motors = act.motors * scale;
motors = min(max(motors, 0), act.motor_max * scale);
if nargin >= 5 && ~isempty(fault_idx) && fault_idx >= 0 && fault_idx <= 3
    motors(fault_idx + 1) = 0;
end
[thrust, torque] = qds.motors_to_wrench(motors, act.hover, act.arm);
end
