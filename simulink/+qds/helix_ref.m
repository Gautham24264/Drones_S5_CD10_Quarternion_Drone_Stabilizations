function [p_des, v_des, a_ff] = helix_ref(t)
warmup = min(t / 3.0, 1.0);
w = 0.55;
r = 1.0;
p_des = [r * cos(w * t) * warmup; r * sin(w * t) * warmup; 1.2 + 0.35 * t * warmup];
v_des = [-r * w * sin(w * t) * warmup; r * w * cos(w * t) * warmup; 0.35 * warmup];
a_ff = [-r * w^2 * cos(w * t) * warmup; -r * w^2 * sin(w * t) * warmup; 0];
end
