function [thrust, q_des] = cascade_position(p_des, p, v, v_des, a_ff, mass, g, kp, kd, max_tilt, yaw_des)
if nargin < 11, yaw_des = 0; end
if nargin < 10, max_tilt = 0.55; end
if nargin < 9, kd = 2.2; end
if nargin < 8, kp = 1.8; end
if nargin < 7, g = 9.81; end
if nargin < 6, mass = 1.0; end
e = p_des(:) - p(:);
a_des = kp * e + kd * (v_des(:) - v(:)) + a_ff(:) + [0; 0; g];
horiz = a_des(1:2);
max_h = g * tan(max_tilt);
nh = norm(horiz);
if nh > max_h
    horiz = horiz * (max_h / nh);
    a_des(1:2) = horiz;
end
a_des(3) = max(a_des(3), 0.3 * g);
thrust = mass * norm(a_des);
q_des = qds.from_two_vectors_yaw(a_des, yaw_des);
end
