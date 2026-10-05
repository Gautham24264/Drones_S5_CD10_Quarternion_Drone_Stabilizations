function [p_des, v_des, a_ff] = figure8_ref(t)
warmup = min(t / 2.5, 1.0);
wx = 0.45; wy = 0.90;
ax_ = 1.4; ay_ = 0.9;
p_des = [ax_ * sin(wx * t) * warmup; ay_ * sin(wy * t) * warmup; 1.5];
v_des = [ax_ * wx * cos(wx * t) * warmup; ay_ * wy * cos(wy * t) * warmup; 0.0];
a_ff = [-ax_ * wx^2 * sin(wx * t) * warmup; -ay_ * wy^2 * sin(wy * t) * warmup; 0.0];
end
