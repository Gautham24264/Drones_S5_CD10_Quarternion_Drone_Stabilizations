function [tau, integ] = pid_attitude(qd, q, omega, dt, kp, ki, kd, integ)
e = qds.vector_error(qd, q);
integ = integ + e*dt;
integ = min(max(integ,-0.4),0.4);
tau = kp*e + ki*integ - kd*omega(:);
end
