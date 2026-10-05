function e = vector_error(qd, q)
qd = qds.normalize(qd(:).');
q = qds.normalize(q(:).');
qc = [q(1), -q(2), -q(3), -q(4)];
qe = qds.multiply(qd, qc);
if qe(1) < 0, qe = -qe; end
e = qe(2:4).';
end
