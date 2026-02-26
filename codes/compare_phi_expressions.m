% Compare different barrier function (phi) expressions side-by-side
%
% Runs the closed-loop simulation once per expression, collects results,
% and plots lane error, safety probability, and speed for comparison.

clear; close all;

%% --- Define phi expressions to compare ---
phi_list = {
    '1 - (e/emax)^2',          'quadratic (default)';
    '1 - (e/emax)^4',          'quartic';
    'cos(pi*e/(2*emax))',       'cosine';
    '1 - abs(e/emax)',          'linear';
};

N = size(phi_list, 1);
colors = lines(N);

%% --- Setup (shared across all runs) ---
addpath impl_controller impl_model impl_estimator impl_road

mdl = 'mdl_closed_loop_mpc';
load_system(mdl)

ONLINE_ESTIMATION = '1';
set_param([mdl '/estimation_sw'],'sw', ONLINE_ESTIMATION)
set_param([mdl '/estimate_fixed'], 'Value',  '[0.30, 0.01]')
set_param([mdl '/prior'],'InitialCondition', '[0.30, 0.01]')
set_param([mdl '/mes_var'], 'Value', '0.1')

EMAX = 3;
set_param([mdl '/SafeProbabilityMC'],'emax',num2str(EMAX))
set_param([mdl '/SafeProbabilityMC'],'snum','100')
set_param([mdl '/visualization'],'Commented','on')

TERM_DIST = '150';
set_param([mdl '/termination_dist'], 'Value', TERM_DIST)
TERM_LAT_ERROR = '100';
set_param([mdl '/termination_lat'], 'Value', TERM_LAT_ERROR)

V0 = 20 * 1000/3600;
Re = 0.325;
INIT_DYN  = [V0, 0, 0, 0];
INIT_OMG  = V0/Re*ones(1,4);
INIT_ROAD = [0,0,0];
init = [INIT_DYN  INIT_OMG 0 INIT_ROAD];
set_param([mdl '/initial_vehicle_state'],'Value', ['[' num2str(init) ']'] )

%% MPC controller (built once, reused for all runs)
disp('--- defining MPC controller ----')
nlobj = nlmpc(12,3,2);
nlobj.Ts = 0.2;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 2;
nlobj.Model.StateFcn  = "fun_system_dynamics";
nlobj.Model.OutputFcn = "fun_output_function";
nlobj.Model.NumberOfParameters = 2;
nlobj.Weights.OutputVariables = [0.03 1 1];
nlobj.Weights.ManipulatedVariablesRate = [1 1];
nlobj.Optimization.CustomIneqConFcn = "fun_inequality";

u0 = [0 0];
ref0 = [40*1000/3600 0 0];
mu = 0.9;
probs = [1, 0, 1, 1, 0];
validateFcns(nlobj,init,u0,{},{mu,probs},ref0);
nloptions = nlmpcmoveopt;
nloptions.Parameters = {mu, probs};
mv = u0; xk = init;
[mv,nloptions] = nlmpcmove(nlobj,xk,mv,ref0,[],nloptions);

code_gen = true;
[coreData,onlineData] = getCodeGenerationData(nlobj,xk',mv,nloptions.Parameters);
if code_gen
    disp('--- building MPC controller ----')
    Cfg = coder.config('mex');
    Cfg.EnableDynamicMemoryAllocation = true;
    Cfg.DynamicMemoryAllocationThreshold = 65536;
    codegen('-config',Cfg,'nlmpcmoveCodeGeneration','-o','fun_mpc_controller','-args',...
        {coder.Constant(coreData), xk', mv, onlineData});
end

%% --- Run simulation for each phi expression ---
mu = 0.3;
set_param([mdl '/true_friction_coeff'],'Value', num2str(mu))

results = cell(N, 1);
for k = 1:N
    fprintf('\n=== Run %d/%d: %s  [phi = %s] ===\n', k, N, phi_list{k,2}, phi_list{k,1});
    set_phi_expr(phi_list{k,1});
    clear fun_safety_condition;
    rehash;
    pctRunOnAll('clear fun_safety_condition; rehash;');

    % Verify the file was actually rewritten
    verify_txt = fileread(which('fun_safety_condition'));
    phi_line = extractBetween(verify_txt, 'phi =', '%#PHI_EXPR');
    fprintf('  Verify file contents: phi =%s\n', strtrim(phi_line{1}));

    res = sim([mdl '.slx']);

    results{k}.prob  = renamevars(res.SafeProb.extractTimetable,    'Data','prob');
    results{k}.state = renamevars(res.State.extractTimetable,       'Data','state');
    results{k}.input = renamevars(res.Input.extractTimetable,       'Data','input');
    results{k}.traj  = renamevars(res.Trajectory.extractTimetable,  'Data','traj');
    results{k}.name  = phi_list{k,2};
    results{k}.expr  = phi_list{k,1};
end

% Restore default after all runs
set_phi_expr('1 - (e/emax)^2');
disp('--- all runs completed ----')

%% --- Plot comparison ---
figure('Color','w','Units','inches','Position',[2 2 12 10]);

% 1) Lane error
subplot(3,1,1); hold on; grid on;
for k = 1:N
    plot(results{k}.state.Time, results{k}.state.state(:,11), ...
        'Color', colors(k,:), 'LineWidth', 1.5);
end
ylabel('Lane Error e [m]');
title('Barrier Function Comparison');
legend(phi_list(:,2), 'Location','best');

% 2) Safety probability
subplot(3,1,2); hold on; grid on;
for k = 1:N
    plot(results{k}.prob.Time, results{k}.prob.prob, ...
        'Color', colors(k,:), 'LineWidth', 1.5);
end
ylabel('Safety Probability P');
legend(phi_list(:,2), 'Location','best');

% 3) Speed
subplot(3,1,3); hold on; grid on;
for k = 1:N
    plot(results{k}.state.Time, results{k}.state.state(:,1)*3.6, ...
        'Color', colors(k,:), 'LineWidth', 1.5);
end
ylabel('Speed [km/h]');
xlabel('Time [s]');
legend(phi_list(:,2), 'Location','best');

%% --- Print summary table ---
fprintf('\n%-25s %10s %10s %10s %10s\n', 'Expression', 'MaxError', 'MeanError', 'MinProb', 'MeanProb');
fprintf('%s\n', repmat('-', 1, 70));
for k = 1:N
    e_vals = results{k}.state.state(:,11);
    p_vals = results{k}.prob.prob;
    fprintf('%-25s %10.3f %10.3f %10.3f %10.3f\n', ...
        results{k}.name, max(abs(e_vals)), mean(abs(e_vals)), min(p_vals), mean(p_vals));
end
