function q = normalize(q)
q = q(:).';
n = norm(q);
if n < 1e-12
    q = [1, 0, 0, 0];
else
    q = q / n;
end
end
