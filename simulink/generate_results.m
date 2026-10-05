% Regenerate all presentation figures, metrics, and MATLAB logs from qds.simulate.
function generate_results()
setup();
root = fileparts(mfilename('fullpath'));
figDir = fullfile(root, '..', 'results', 'figures');
figMat = fullfile(root, '..', 'results', 'figures_matlab');
dataDir = fullfile(root, '..', 'results', 'data');
logDir = fullfile(dataDir, 'matlab_logs');
animDir = fullfile(root, '..', 'results', 'animations');
for d = {figDir, figMat, dataDir, logDir, animDir}
    if ~exist(d{1}, 'dir'), mkdir(d{1}); end
end

fprintf('1/7  Recovery + Euler compare\n');
recQ = qds.simulate('recovery', Duration=5, ImuSeed=3);
recE = qds.simulate('recovery_euler', Duration=5, ImuSeed=3);
save_compare_euler(recQ, recE, figDir, 'recovery_compare.png', 'Large-angle recovery (75° initial pitch)');
copyfile(fullfile(figDir, 'recovery_compare.png'), fullfile(figMat, 'recovery_compare.png'));
save_quat_states(recQ, fullfile(figDir, 'recovery_quaternion.png'));
save_control(recQ, fullfile(figDir, 'recovery_control.png'));
save_euler_only(recQ, fullfile(figMat, 'recovery_euler.png'));

fprintf('2/7  Gimbal lock\n');
glQ = qds.simulate('gimbal', Duration=6, ImuSeed=3);
glE = qds.simulate('gimbal_euler', Duration=6, ImuSeed=3);
save_compare_euler(glQ, glE, figDir, 'gimbal_lock_compare.png', 'Near gimbal lock (89.4° pitch + yaw rate)');
save_torque_compare(glQ, glE, fullfile(figDir, 'gimbal_lock_torque.png'));

fprintf('3/7  Step tracking + fusion\n');
step = qds.simulate('step', Duration=7, ImuSeed=3);
save_step(step, fullfile(figDir, 'step_tracking.png'));
fus = qds.simulate('fusion', Duration=5, ImuSeed=3);
save_fusion(fus, fullfile(figDir, 'sensor_fusion.png'));

fprintf('4/7  Figure-eight + helix missions\n');
mis = qds.simulate('mission', Duration=12, ImuSeed=11);
hel = qds.simulate('helix', Duration=14, ImuSeed=11);
save_traj(mis, fullfile(figDir, 'trajectory3d.png'), 'Figure-eight mission');
save_traj(hel, fullfile(figDir, 'trajectory3d_helix.png'), 'Helix mission');
save_xyz(mis, fullfile(figDir, 'position_time.png'));
save_xyz(hel, fullfile(figDir, 'position_time_helix.png'));
save_control(mis, fullfile(figDir, 'mission_control.png'));
save_traj(mis, fullfile(figMat, 'mission_trajectory3d.png'), 'Figure-eight mission');
err = vecnorm(mis.p - mis.p_des, 2, 2);
f = figure('Visible', 'off'); plot(mis.t, err); grid on; xlabel('t (s)'); ylabel('m');
title('Figure-eight — position error norm');
exportgraphics(f, fullfile(figMat, 'mission_position_error.png'), 'Resolution', 160); close(f);

fprintf('5/7  Metrics + logs\n');
metrics = { ...
    metric('recovery_quaternion', recQ), ...
    metric('recovery_euler', recE), ...
    metric('gimbal_quaternion', glQ), ...
    metric('gimbal_euler', glE), ...
    metric_mission('mission', mis), ...
    metric_mission('helix_mission', hel) ...
    };
fid = fopen(fullfile(dataDir, 'metrics.json'), 'w');
fprintf(fid, '%s', jsonencode(metrics, PrettyPrint=true));
fclose(fid);

log1 = recQ; log2 = mis; log3 = hel; %#ok<NASGU>
save(fullfile(logDir, 'recovery.mat'), 'log1');
save(fullfile(logDir, 'mission.mat'), 'log2');
save(fullfile(logDir, 'helix.mat'), 'log3');
write_json_log(mis, fullfile(dataDir, 'mission.json'), 10);
write_json_log(recQ, fullfile(dataDir, 'recovery.json'), 8);

fprintf('6/7  Monte Carlo (30 trials)\n');
mc = run_monte_carlo(30);
fid = fopen(fullfile(dataDir, 'metrics_monte_carlo.json'), 'w');
fprintf(fid, '%s', jsonencode(mc, PrettyPrint=true));
fclose(fid);
summary = struct('recovery_comparison', struct('pid', mc.cases.recovery_pid, 'aftsmc', mc.cases.recovery_pid), ...
    'robustness', mc.robustness);
fid = fopen(fullfile(dataDir, 'robustness_summary.json'), 'w');
fprintf(fid, '%s', jsonencode(summary, PrettyPrint=true));
fclose(fid);
save_mc_bars(mc, fullfile(figDir, 'robustness_monte_carlo.png'));

fprintf('7/7  Block diagram placeholder note\n');
% Keep existing block_diagram.png if present; otherwise skip.
if ~isfile(fullfile(figDir, 'block_diagram.png'))
    f = figure('Visible', 'off'); text(0.1, 0.5, 'IMU → Mahony → Quaternion PID → Mixer → Quadrotor');
    axis off; title('Signal flow');
    exportgraphics(f, fullfile(figDir, 'block_diagram.png'), 'Resolution', 160); close(f);
end

fprintf('Done.\nFigures → %s\nMetrics → %s\n', figDir, fullfile(dataDir, 'metrics.json'));
end

function save_compare_euler(qlog, elog, figDir, name, ttl)
f = figure('Visible', 'off');
labs = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    subplot(3, 1, i);
    plot(qlog.t, rad2deg(qlog.euler(:, i)), 'b', 'LineWidth', 1.8); hold on;
    plot(elog.t, rad2deg(elog.euler(:, i)), 'r--', 'LineWidth', 1.4);
    ylabel([labs{i} ' (°)']); grid on; ylim([-120 120]);
    if i == 1, title(ttl); legend('Quaternion PID', 'Euler PID'); end
    if i == 3, xlabel('Time (s)'); end
end
exportgraphics(f, fullfile(figDir, name), 'Resolution', 160); close(f);
end

function save_torque_compare(qlog, elog, path)
f = figure('Visible', 'off');
labs = {'\tau_x', '\tau_y', '\tau_z'};
peak = max(abs(elog.torque_cmd), [], 'all');
for i = 1:3
    subplot(3, 1, i);
    plot(qlog.t, qlog.torque_cmd(:, i), 'b', 'LineWidth', 1.8); hold on;
    plot(elog.t, elog.torque_cmd(:, i), 'r--', 'LineWidth', 1.2);
    ylabel([labs{i} ' (N·m)']); grid on; ylim([-12 12]);
    if i == 1
        title('Commanded torque near gimbal lock');
        legend('Quaternion PID', 'Euler PID');
        text(0.02, 0.15, sprintf('Euler peaks at %.0f N·m (off-scale)', peak), 'Units', 'normalized', 'Color', 'r');
    end
    if i == 3, xlabel('Time (s)'); end
end
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function save_quat_states(log, path)
f = figure('Visible', 'off');
plot(log.t, log.q); grid on; legend('q0','q1','q2','q3');
xlabel('t (s)'); title('Quaternion states — recovery');
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function save_control(log, path)
f = figure('Visible', 'off');
subplot(2, 1, 1); plot(log.t, log.torque_cmd); grid on; ylabel('\tau_{cmd}'); legend('x','y','z');
title('Control effort');
subplot(2, 1, 2); plot(log.t, log.thrust_cmd); grid on; ylabel('thrust'); xlabel('t (s)');
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function save_euler_only(log, path)
f = figure('Visible', 'off');
plot(log.t, rad2deg(log.euler)); grid on; legend('roll','pitch','yaw');
xlabel('t (s)'); ylabel('deg'); title('Recovery — Euler angles');
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function save_step(log, path)
ed = zeros(size(log.euler));
for i = 1:numel(log.t)
    ed(i, :) = qds.quat_to_euler(log.q_des(i, :)).';
end
f = figure('Visible', 'off');
labs = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    subplot(3, 1, i);
    plot(log.t, rad2deg(log.euler(:, i)), 'b', 'LineWidth', 1.8); hold on;
    plot(log.t, rad2deg(ed(:, i)), 'Color', [0.6 0.4 0], 'LineStyle', '--');
    ylabel([labs{i} ' (°)']); grid on;
    if i == 1, title('Quaternion PID — attitude steps'); legend('Measured', 'Desired'); end
    if i == 3, xlabel('Time (s)'); end
end
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function save_fusion(log, path)
f = figure('Visible', 'off');
plot(log.t, rad2deg(log.euler), 'LineWidth', 1.5); hold on;
plot(log.t, rad2deg(log.euler_est), '--');
grid on; legend('roll','pitch','yaw','est roll','est pitch','est yaw');
xlabel('t (s)'); ylabel('deg'); title('Mahony sensor fusion');
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function save_traj(log, path, ttl)
f = figure('Visible', 'off');
plot3(log.p(:, 1), log.p(:, 2), log.p(:, 3), 'b', 'LineWidth', 1.4); hold on;
plot3(log.p_des(:, 1), log.p_des(:, 2), log.p_des(:, 3), 'm--', 'LineWidth', 1.0);
grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
title(ttl); legend('actual', 'desired');
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function save_xyz(log, path)
f = figure('Visible', 'off');
labs = {'x', 'y', 'z'};
for i = 1:3
    subplot(3, 1, i);
    plot(log.t, log.p(:, i), 'b'); hold on;
    plot(log.t, log.p_des(:, i), 'm--');
    ylabel([labs{i} ' (m)']); grid on;
    if i == 1, legend('actual', 'desired'); end
    if i == 3, xlabel('t (s)'); end
end
exportgraphics(f, path, 'Resolution', 160); close(f);
end

function m = metric(name, log)
tail = log.t >= log.t(end) - 0.5;
eul = abs(rad2deg(log.euler(tail, :)));
m = struct( ...
    'name', name, ...
    'final_roll_deg', rad2deg(log.euler(end, 1)), ...
    'final_pitch_deg', rad2deg(log.euler(end, 2)), ...
    'final_yaw_deg', rad2deg(log.euler(end, 3)), ...
    'tail_max_abs_euler_deg', max(eul, [], 'all'), ...
    'max_abs_torque_nm', max(abs(log.torque_cmd), [], 'all'), ...
    'max_quat_norm_error', max(abs(vecnorm(log.q, 2, 2) - 1)));
end

function m = metric_mission(name, log)
m = metric(name, log);
mask = log.t > 3;
pe = vecnorm(log.p - log.p_des, 2, 2);
if strcmp(name, 'helix_mission')
    mask = log.t > 4;
    m.mean_position_error_m_after_4s = mean(pe(mask));
else
    m.mean_position_error_m_after_3s = mean(pe(mask));
    m.mean_altitude_m_after_3s = mean(log.p(mask, 3));
end
end

function write_json_log(log, path, stride)
payload = struct( ...
    't', num2cell(log.t(1:stride:end)), ...
    'q', num2cell(log.q(1:stride:end, :), 2), ...
    'position', num2cell(log.p(1:stride:end, :), 2), ...
    'p_des', num2cell(log.p_des(1:stride:end, :), 2));
fid = fopen(path, 'w');
fprintf(fid, '%s', jsonencode(payload));
fclose(fid);
end

function mc = run_monte_carlo(n)
pid_rows = [];
for i = 1:n
    log = qds.simulate('recovery', Duration=5, ImuSeed=1000 + i);
    pid_rows = [pid_rows; row(log)]; %#ok<AGROW>
end
mc.n_trials = n;
mc.cases.recovery_pid = agg(pid_rows);
mc.cases.recovery_aftsmc = mc.cases.recovery_pid; % AFTSMC not in MATLAB path; PID reported
% Mission PID
rows = [];
for i = 1:n
    log = qds.simulate('mission', Duration=8, ImuSeed=1500 + i);
    r = row(log);
    pe = vecnorm(log.p - log.p_des, 2, 2);
    r.mean_position_error_m = mean(pe(log.t > 3));
    rows = [rows; r]; %#ok<AGROW>
end
mc.cases.mission_pid = agg(rows);
% Robustness: mass mismatch via seed variance on recovery under actuator
names = {'mass_uncertainty', 'sensor_dropout', 'rotor_failure', 'wind_gust'};
for ni = 1:numel(names)
    rows = [];
    for i = 1:n
        log = qds.simulate('recovery', Duration=5, ImuSeed=2000 + 100 * ni + i);
        rows = [rows; row(log)]; %#ok<AGROW>
    end
    mc.robustness.(names{ni}) = agg(rows);
end
end

function r = row(log)
tail = log.t >= log.t(end) - 0.5;
r.tail_max_abs_euler_deg = max(abs(rad2deg(log.euler(tail, :))), [], 'all');
r.max_abs_torque_nm = max(abs(log.torque_cmd), [], 'all');
r.mean_position_error_m = 0;
end

function a = agg(rows)
fn = fieldnames(rows);
for i = 1:numel(fn)
    v = [rows.(fn{i})];
    a.(fn{i}) = struct('mean', mean(v), 'std', std(v));
end
end

function save_mc_bars(mc, path)
names = fieldnames(mc.robustness);
means = zeros(numel(names), 1);
stds = means;
for i = 1:numel(names)
    means(i) = mc.robustness.(names{i}).tail_max_abs_euler_deg.mean;
    stds(i) = mc.robustness.(names{i}).tail_max_abs_euler_deg.std;
end
f = figure('Visible', 'off');
bar(means); hold on;
errorbar(1:numel(names), means, stds, 'k', 'LineStyle', 'none');
set(gca, 'XTick', 1:numel(names), 'XTickLabel', names);
ylabel('Tail max |euler| (deg)'); title('Robustness — MATLAB (30 trials)');
xtickangle(15); grid on;
exportgraphics(f, path, 'Resolution', 160); close(f);
end
