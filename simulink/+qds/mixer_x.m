function motors = mixer_x(thrust, torque, hover, arm)
tau = torque(:).';
roll = tau(1)/max(arm,1e-6);
pitch = tau(2)/max(arm,1e-6);
yaw = tau(3);
motors = [thrust/4+roll+pitch-yaw; thrust/4-roll+pitch+yaw; thrust/4-roll-pitch-yaw; thrust/4+roll-pitch+yaw];
motors = min(max(motors,0), hover*2.5);
end
