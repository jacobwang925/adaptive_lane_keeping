% Compare different barrier function (phi) expressions side-by-side
%
% Calls main_single_run.m for each expression, collects results,
% and plots lane error, safety probability, and speed for comparison.
%
% IMPORTANT: The impact of phi expressions depends on the SAFETY_METHOD:
%   - 'PSC' (Proposed): Phi expressions MATTER. The safety condition is used
%     in Monte Carlo simulations to calculate safety probability (p, LfP, LgP, BP),
%     which are then used in the PSC constraint.
%   - 'CDBF': Phi expressions are IGNORED. Uses control-dependent barrier
%     functions based on vehicle dynamics instead.
%   - 'DIRECT': Phi expressions are IGNORED. Uses hardcoded lane error constraint.
%   - 'NONE' (AMPC): Phi expressions are IGNORED. No safety constraints.
%
% For testing how different phi expressions impact safety scores, use 'PSC'.

clear; close all;

%% --- Define phi expressions to compare ---
% Format: {'expression', 'display_name'; ...}
% The expression will be substituted into fun_safety_condition.m
%
% Available variables in expression:
%   e     - lateral error (state variable)
%   emax  - maximum allowed error (set by EMAX parameter)
%
% Example expressions:
%   '1 - (e/emax)^2'          - Quadratic (default), smooth, moderate
%   '1 - (e/emax)^4'          - Quartic, flatter near center, steeper near boundary
%   'cos(pi*e/(2*emax))'      - Cosine, very smooth at boundary
%   '1 - abs(e/emax)'         - Linear, constant slope, sharp corner at e=0
phi_list = {
    '1 - (e/emax)^2',          'quadratic (default)';
    '1 - (e/emax)^4',          'quartic';
    'cos(pi*e/(2*emax))',       'cosine';
    '1 - abs(e/emax)',          'linear';
};

N = size(phi_list, 1);
colors = lines(N);

%% --- Setup paths ---
addpath impl_controller impl_model impl_estimator impl_road

%% --- Common parameters for all runs ---
EMAX = 3;                          % Lane error tolerance [m]
SAFETY_METHOD = 'DIRECT';          % SAFETY METHOD: 'PSC', 'CDBF', 'DIRECT', or 'NONE'
MU_VALUE = 0.3;                    % Friction coefficient (0.3 = icy, 0.9 = dry)

%% --- Run simulation for each phi expression ---
results = cell(N, 1);

for k = 1:N
    fprintf('\n=== Run %d/%d: %s  [phi = %s] ===\n', k, N, phi_list{k,2}, phi_list{k,1});

    % Call main_single_run with current phi expression
    % Arguments: phi_expr, emax, safety_method, mu_value
    res = main_single_run(phi_list{k,1}, EMAX, SAFETY_METHOD, MU_VALUE);

    % Store results
    results{k} = res;
    results{k}.name = phi_list{k,2};
    results{k}.expr = phi_list{k,1};

    fprintf('  Completed: Max error = %.3f, Min prob = %.3f\n', ...
        max(abs(res.state.state(:,11))), min(res.prob.prob));
end

% Restore default phi expression after all runs
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
title(sprintf('Barrier Function Comparison (Safety Method: %s)', SAFETY_METHOD));
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
