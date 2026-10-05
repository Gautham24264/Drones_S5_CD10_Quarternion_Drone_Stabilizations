function A = allocation_matrix(arm)
r = 1.0 / max(arm, 1e-6);
A = [1, 1, -1; -1, 1, 1; -1, -1, -1; 1, -1, 1] * r;
end
