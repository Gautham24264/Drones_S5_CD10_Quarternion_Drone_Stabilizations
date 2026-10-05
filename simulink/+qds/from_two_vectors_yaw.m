function q = from_two_vectors_yaw(body_z, yaw)
b3 = body_z(:);
n = norm(b3);
if n < 1e-12
    b3 = [0; 0; 1];
else
    b3 = b3 / n;
end
c = cos(yaw); s = sin(yaw);
b1_des = [c; s; 0];
b2 = cross(b3, b1_des);
n2 = norm(b2);
if n2 < 1e-6
    b1_des = [-s; c; 0];
    b2 = cross(b3, b1_des);
    n2 = norm(b2);
end
b2 = b2 / n2;
b1 = cross(b2, b3);
R = [b1 b2 b3];
q = qds.normalize(rotm_to_quat(R));
end

function q = rotm_to_quat(R)
tr = trace(R);
if tr > 0
    s = sqrt(tr + 1) * 2;
    q = [0.25 * s; (R(3, 2) - R(2, 3)) / s; (R(1, 3) - R(3, 1)) / s; (R(2, 1) - R(1, 2)) / s];
else
    q = [1; 0; 0; 0];
end
q = qds.normalize(q.');
end
