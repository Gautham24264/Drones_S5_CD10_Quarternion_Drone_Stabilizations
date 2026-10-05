% Add simulink and +qds to the MATLAB path (run once per session).
function setup()
root = fileparts(mfilename('fullpath'));
addpath(root);
fprintf('Quaternion drone simulation ready.\n');
fprintf('  live_demo(''mission'')  — real-time figure-eight\n');
fprintf('  live_demo(''recovery'') — real-time 75° recovery\n');
fprintf('  run_scenarios          — batch logs\n');
fprintf('  generate_results       — figures to results/figures_matlab\n');
end
