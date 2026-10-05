function eul = quat_to_euler(q)
q = qds.normalize(q(:).');
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
sinp = max(min(2 * (q0 * q2 - q3 * q1), 1), -1);
pitch = asin(sinp);
roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1^2 + q2^2));
yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2^2 + q3^2));
eul = [roll; pitch; yaw];
end
