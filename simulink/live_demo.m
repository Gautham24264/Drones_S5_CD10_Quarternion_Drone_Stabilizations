% Real-time 3D visualization — primary project demo (MATLAB).
% Usage: setup; live_demo('mission') | live_demo('recovery') | live_demo('helix')
function live_demo(scenario)
if nargin < 1, scenario = 'mission'; end
setup();
T = 12.0;
if strcmpi(scenario, 'recovery'), T = 5.0; end
if strcmpi(scenario, 'helix'), T = 14.0; end

title_str = ['Quaternion UAV — ' scenario];
fig = figure('Name', title_str, 'Color', 'w');
ax = axes(fig);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
view(ax, 35, 25);
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
title(ax, title_str);
hTrail = plot3(ax, nan, nan, nan, 'Color', [0.55 0.58 0.61], 'LineWidth', 1);
hArmX = plot3(ax, nan, nan, nan, 'r', 'LineWidth', 2.5);
hArmY = plot3(ax, nan, nan, nan, 'b', 'LineWidth', 2.5);
hUp = plot3(ax, nan, nan, nan, 'g', 'LineWidth', 1.5);
hRot = plot3(ax, nan, nan, nan, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', [0.15 0.15 0.15]);
hDes = plot3(ax, nan, nan, nan, 'm--', 'LineWidth', 1.0);
tLabel = text(ax, 0.02, 0.95, 0, '', 'Units', 'normalized');
trail = zeros(0, 3);
t0 = tic;
speedup = 1.0;

cb = @(t, p, q) on_frame(t, p, q);
qds.simulate(lower(scenario), Duration=T, LiveCallback=cb, Stride=8);

    function stop = on_frame(t, p, q)
        stop = ~ishandle(fig);
        if stop, return; end
        arm = 0.28;
        R = qds.rotation_matrix(q);
        o = p(:);
        xa = o + R * [-arm; 0; 0]; xb = o + R * [arm; 0; 0];
        ya = o + R * [0; -arm; 0]; yb = o + R * [0; arm; 0];
        up = o + R * [0; 0; 0.18];
        rr = [1 1; -1 1; -1 -1; 1 -1] * (arm * 0.7);
        rot = zeros(3, 4);
        for j = 1:4
            rot(:, j) = o + R * [rr(j, 1); rr(j, 2); 0];
        end
        trail(end + 1, :) = o.'; %#ok<AGROW>
        set(hTrail, 'XData', trail(:, 1), 'YData', trail(:, 2), 'ZData', trail(:, 3));
        set(hArmX, 'XData', [xa(1) xb(1)], 'YData', [xa(2) xb(2)], 'ZData', [xa(3) xb(3)]);
        set(hArmY, 'XData', [ya(1) yb(1)], 'YData', [ya(2) yb(2)], 'ZData', [ya(3) yb(3)]);
        set(hUp, 'XData', [o(1) up(1)], 'YData', [o(2) up(2)], 'ZData', [o(3) up(3)]);
        set(hRot, 'XData', rot(1, :), 'YData', rot(2, :), 'ZData', rot(3, :));
        set(tLabel, 'String', sprintf('t = %.2f s', t));
        if strcmpi(scenario, 'mission') || strcmpi(scenario, 'helix')
            [pd, ~, ~] = path_ref(lower(scenario), t);
            set(hDes, 'XData', pd(1), 'YData', pd(2), 'ZData', pd(3));
        end
        c = o.';
        r = 2.0;
        if size(trail, 1) > 10
            r = max(max(trail) - min(trail)) * 0.55 + 1.5;
            c = mean(trail(max(1, end - 200):end, :), 1);
        end
        xlim(ax, [c(1) - r, c(1) + r]);
        ylim(ax, [c(2) - r, c(2) + r]);
        zlim(ax, [max(0, c(3) - r * 0.8), c(3) + r * 0.8]);
        drawnow limitrate;
        elapsed = toc(t0);
        target = t / speedup;
        if target > elapsed, pause(target - elapsed); end
    end
end

function [p, v, a] = path_ref(name, t)
if strcmp(name, 'helix')
    [p, v, a] = qds.helix_ref(t);
else
    [p, v, a] = qds.figure8_ref(t);
end
end
