function [filt, q_hat] = mahony_update(filt, gyro, accel, dt, mag)
gyro = gyro(:);
accel = accel(:);
err = zeros(3, 1);
an = norm(accel);
if an > 1e-6
    v_acc = accel / an;
    v_est = qds.rotation_matrix(filt.q).'*[0; 0; 1];
    err = err + cross(v_acc, v_est);
end
use_mag = nargin >= 5 && ~isempty(mag);
if use_mag
    mn = norm(mag);
    if mn > 1e-6
        v_mag = mag / mn;
        fn = norm(filt.field);
        v_est_m = qds.rotation_matrix(filt.q).' * (filt.field / max(fn, 1e-9));
        err = err + cross(v_mag, v_est_m);
    end
end
if norm(err) > 0
    filt.integ = filt.integ + err * dt;
    gain = filt.kp + filt.km * double(use_mag);
    omega = gyro + gain * err + filt.ki * filt.integ;
else
    omega = gyro;
end
filt.q = qds.integrate(filt.q, omega, dt);
q_hat = filt.q;
end
