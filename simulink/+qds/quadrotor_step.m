function [state, last_accel] = quadrotor_step(state, thrust, torque, dt, wind, attitude_only)
% state = [p(3); v(3); q(4); omega(3)]  (scalar-first quaternion)
mass = 1.0;
g = 9.81;
J = diag([0.012, 0.012, 0.021]);
Jinv = inv(J);
max_thrust = 25;
max_tau = 4;
if nargin < 6, attitude_only = false; end
if nargin < 5 || isempty(wind), wind = zeros(3, 1); end

p = state(1:3);
v = state(4:6);
q = state(7:10);
w = state(11:13);
thrust = min(max(thrust, 0), max_thrust);
torque = min(max(torque(:), -max_tau), max_tau);

R = qds.rotation_matrix(q);
last_accel = [0; 0; -g] + R * [0; 0; thrust / mass] + wind(:);
if attitude_only
    v(:) = 0;
else
    v = v + last_accel * dt;
    p = p + v * dt;
    if p(3) < 0
        p(3) = 0;
        if v(3) < 0, v(3) = 0; end
    end
end
Jw = J * w;
wd = Jinv * (-cross(w, Jw) + torque);
w = w + wd * dt;
q = qds.integrate(q, w, dt);
state = [p(:); v(:); q(:); w(:)];
end
