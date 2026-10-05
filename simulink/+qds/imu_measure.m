function [gyro, accel, mag] = imu_measure(imu, q, omega, accel_inertial, dropout)
R = qds.rotation_matrix(q);
g_vec = [0; 0; -imu.g];
accel_true = R.' * (accel_inertial(:) - g_vec);
mag_true = R.' * imu.field;
gyro = omega(:) + imu.bias + imu.gyro_std * randn(3, 1);
if nargin >= 5 && dropout
    accel = zeros(3, 1);
    mag = zeros(3, 1);
else
    accel = accel_true + imu.accel_std * randn(3, 1);
    mag = mag_true + imu.mag_bias + imu.mag_std * randn(3, 1);
end
end
