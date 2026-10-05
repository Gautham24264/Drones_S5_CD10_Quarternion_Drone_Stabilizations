function [tau, integ] = pid_attitude_euler(qd, q, omega, dt, kp, ki, kd, integ)
% Euler-angle PID with kinematic Jacobian inversion (gimbal-lock baseline).
eul_d = qds.quat_to_euler(qd);
eul = qds.quat_to_euler(q);
err = wrap(eul_d - eul);
integ = integ + err * dt;
integ = min(max(integ, -0.4), 0.4);
u = kp * err + ki * integ;
roll = eul(1); pitch = eul(2);
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
E = [1, 0, -sp; 0, cr, sr * cp; 0, -sr, cr * cp];
if abs(det(E)) < 1e-6
    tau = pinv(E) * u;
else
    tau = E \ u;
end
tau = tau - kd * omega(:);
end

function a = wrap(a)
a = mod(a + pi, 2 * pi) - pi;
end
