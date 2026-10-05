function q = integrate(q, omega, dt)
omega = omega(:);
angle = norm(omega)*dt;
if angle < 1e-12
    dq = [1,0,0,0];
else
    axis = omega/norm(omega);
    half = 0.5*angle;
    dq = [cos(half), axis.'*sin(half)];
end
q = qds.normalize(qds.multiply(q, dq));
end
