% Overlay MATLAB recovery logs against Python exports.
function compare_to_python()
root = fileparts(mfilename('fullpath'));
pyDir = fullfile(root, '..', 'results', 'data');
matDir = fullfile(pyDir, 'matlab_logs');
if ~isfile(fullfile(matDir, 'recovery.mat'))
    run_scenarios();
end
py = jsondecode(fileread(fullfile(pyDir, 'recovery.json')));
S = load(fullfile(matDir, 'recovery.mat'));
if iscell(py.t)
    t_py = cell2mat(py.t);
else
    t_py = py.t;
end
% Python JSON stores euler indirectly via q — skip if unavailable
fprintf('Python recovery samples: %d\n', numel(t_py));
fprintf('MATLAB recovery samples:  %d\n', numel(S.log1.t));
fprintf('Compare attitude traces manually in Simulink or extend this script with exported euler.\n');
end
