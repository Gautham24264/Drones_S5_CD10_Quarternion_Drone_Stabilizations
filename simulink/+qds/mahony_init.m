function filt = mahony_init(q0, kp, ki, km, field_inertial)
if nargin < 5, field_inertial = [0.5; 0; 0.35]; end
filt.q = qds.normalize(q0(:).');
filt.integ = zeros(3, 1);
filt.kp = kp;
filt.ki = ki;
filt.km = km;
filt.field = field_inertial(:);
end
