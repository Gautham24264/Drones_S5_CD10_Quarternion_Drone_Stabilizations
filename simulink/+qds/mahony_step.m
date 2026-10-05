function q = mahony_step(q, gyro, accel, mag, dt, kp, ki, km, integ)
gyro = gyro(:); accel = accel(:);
err = zeros(3,1);
an = norm(accel);
if an > 1e-6
    v_acc = accel/an;
    R = qds.rotation_matrix(q);
    v_est = R.'*[0;0;1];
    err = err + cross(v_acc, v_est);
end
if nargin >= 4 && ~isempty(mag)
    mn = norm(mag);
    if mn > 1e-6
        v_mag = mag/mn;
        field = [0.5;0;0.35]; field = field/norm(field);
        v_est_m = qds.rotation_matrix(q).'*field;
        err = err + cross(v_mag, v_est_m);
    end
end
integ = integ + err*dt;
gain = kp + (km * double(~isempty(mag)));
omega = gyro + gain*err + ki*integ;
q = qds.integrate(q, omega, dt);
end
