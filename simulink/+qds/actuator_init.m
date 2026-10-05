function act = actuator_init(hover, arm, tau_esc)
if nargin < 1, hover = 9.81; end
if nargin < 2, arm = 0.2; end
if nargin < 3, tau_esc = 0.025; end
act.hover = hover;
act.arm = arm;
act.tau_esc = tau_esc;
act.v0 = 12.0;
act.v_min = 9.0;
act.sag_k = 0.015;
act.motor_max = hover * 2.5;
act.motors = ones(4, 1) * (hover / 4);
end
