% Batch simulations → results/data/matlab_logs/
function run_scenarios()
setup();
out = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'data', 'matlab_logs');
if ~exist(out, 'dir'), mkdir(out); end

fprintf('Recovery...\n');
log1 = qds.simulate('recovery', Duration=5, ImuSeed=3);
save(fullfile(out, 'recovery.mat'), 'log1');

fprintf('Mission...\n');
log2 = qds.simulate('mission', Duration=12, ImuSeed=11);
save(fullfile(out, 'mission.mat'), 'log2');

fprintf('Helix...\n');
log3 = qds.simulate('helix', Duration=14, ImuSeed=11);
save(fullfile(out, 'helix.mat'), 'log3');

fprintf('Logs saved to %s\n', out);
end
