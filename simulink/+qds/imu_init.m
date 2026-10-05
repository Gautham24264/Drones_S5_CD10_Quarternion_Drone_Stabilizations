function imu = imu_init(seed, gyro_std, accel_std, mag_std)
if nargin < 1, seed = 7; end
if nargin < 2, gyro_std = 0.01; end
if nargin < 3, accel_std = 0.15; end
if nargin < 4, mag_std = 0.02; end
imu.g = 9.81;
imu.gyro_std = gyro_std;
imu.accel_std = accel_std;
imu.mag_std = mag_std;
imu.bias = [0.008; -0.006; 0.004];
imu.mag_bias = zeros(3, 1);
imu.field = [0.5; 0; 0.35];
imu.seed = seed;
rng(seed, 'twister');
end
