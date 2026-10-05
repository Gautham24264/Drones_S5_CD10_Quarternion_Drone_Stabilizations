% Build quadrotor_closed_loop.slx (ode4, fixed step 0.002 s).
% Requires Simulink. Simscape Multibody plant block is added when licensed.
function build_quadrotor_model()
root = fileparts(mfilename('fullpath'));
addpath(root);
mdl = 'quadrotor_multibody_plant';
if bdIsLoaded(mdl), close_system(mdl,0); end
if isfile(fullfile(root, [mdl '.slx'])), delete(fullfile(root, [mdl '.slx'])); end

new_system(mdl);
open_system(mdl);

set_param(mdl, 'Solver', 'ode4', 'FixedStep', '0.002', 'StopTime', '5');

% Minimal closed-loop skeleton: thrust/tau inputs -> plant integrator states
add_block('simulink/Sources/In1', [mdl '/thrust'], 'Position', [40 40 70 60]);
add_block('simulink/Sources/In1', [mdl '/tau'], 'Position', [40 100 70 120]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/plant_step'], ...
    'Position', [160 50 260 120]);
add_block('simulink/Sinks/Out1', [mdl '/state'], 'Position', [340 70 370 90]);

set_param([mdl '/tau'], 'Port', '2');

if license('test', 'Simscape_Multibody')
    note = 'Simscape Multibody: replace plant_step with 6-DOF body (m=1, I=diag(0.012,0.012,0.021)).';
else
    note = 'Simscape Multibody license not active — using MATLAB Function plant_step (matches Python ODE).';
end
fprintf('%s\n', note);

save_system(mdl, fullfile(root, [mdl '.slx']));
fprintf('%s\n', note);
fprintf('Saved %s.slx\n', fullfile(root, mdl));
end
