% Compare barrier function (phi) expressions across all safety methods
%
% Runs main_single_run for every pair (safety method × phi row), then saves
% codes/data_mpc/phi_comparison_results.mat. Plot with plot_phi_comparison.m (PSC lateral
% error, overlapping phi curves).
%
% Saved variables: results_grid, safety_method_list, phi_list, EMAX, MU_VALUE
%   results_grid{m,k} — result for safety_method_list{m} and phi_list(k,:)
%
% IMPORTANT: The impact of phi expressions depends on the SAFETY_METHOD:
%   - 'DIRECT': Phi expressions MATTER (constraint shape).
%   - 'PSC': Uses fun_safety_condition; phi shape may or may not change MC stats.
%   - 'CDBF' / 'NONE': Phi expressions are IGNORED — curves for different phi
%     should overlap aside from numerical effects.
%
% NOTE on expression stability with DIRECT:
%   Non-smooth or steep phi can make the MPC solver diverge; prefer smooth phis.

clear; close all;

thisDir = fileparts(mfilename('fullpath'));   % .../codes/phi
codesDir = fileparts(thisDir);               % .../codes
addpath(thisDir, codesDir, ...
    fullfile(codesDir, 'impl_controller'), ...
    fullfile(codesDir, 'impl_model'), ...
    fullfile(codesDir, 'impl_estimator'), ...
    fullfile(codesDir, 'impl_road'));

DATA_FILE = fullfile(codesDir, 'data_mpc', 'phi_comparison_results.mat');

%% --- Safety methods (order = run order) ---
safety_method_list = {'PSC'; 'CDBF'; 'DIRECT'; 'NONE'};

%% --- Define phi expressions to compare ---
% Format: {'expression', 'display_name'; ...}
phi_list = {
    '1 - (e/emax)^2',          'quadratic (default)';
    '1 - (e/emax)^4',          'quartic';
    'cos(pi*e/(2*emax))',       'cosine';
    '1 - abs(e/emax)',          'linear';
};

M = numel(safety_method_list);
N = size(phi_list, 1);

%% --- Common parameters for all runs ---
EMAX = 3;                          % Lane error tolerance [m]
MU_VALUE = 0.3;                    % Friction coefficient (0.3 = icy, 0.9 = dry)

results_grid = cell(M, N);
total_runs = M * N;
run_id = 0;

for mi = 1:M
    sm = char(safety_method_list{mi});
    for k = 1:N
        run_id = run_id + 1;
        fprintf('\n=== Run %d/%d  method=%s  phi=%s (%s) ===\n', ...
            run_id, total_runs, sm, phi_list{k,2}, phi_list{k,1});

        res = main_single_run(phi_list{k,1}, EMAX, sm, MU_VALUE);

        res.name = phi_list{k,2};
        res.expr = phi_list{k,1};
        res.safety_method = sm;
        results_grid{mi, k} = res;

        fprintf('  Completed: Max error = %.3f, Min prob = %.3f\n', ...
            max(abs(res.state.state(:,11))), min(res.prob.prob));
    end
end

set_phi_expr('1 - (e/emax)^2');
disp('--- all runs completed ----')

d = fileparts(DATA_FILE);
if ~isempty(d) && ~isfolder(d)
    mkdir(d);
end
save(DATA_FILE, 'results_grid', 'phi_list', 'safety_method_list', 'EMAX', 'MU_VALUE', '-v7.3');
fprintf('Saved: %s\nPlot: run(''phi/plot_phi_comparison.m'') from the codes/ directory, or open plot_phi_comparison.m in codes/phi/.\n', DATA_FILE);
