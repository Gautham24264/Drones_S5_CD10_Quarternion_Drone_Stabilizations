function [thrust, tau] = motors_to_wrench(motors, hover, arm)
m = motors(:);
thrust = sum(m);
A = qds.allocation_matrix(arm);
dm = m - thrust / 4;
tau = A \ dm;
tau = tau(:);
end
