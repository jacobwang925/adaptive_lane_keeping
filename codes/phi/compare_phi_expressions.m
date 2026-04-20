% Compare barrier function (phi) expressions under PSC only
%
% Runs main_single_run once per phi row with safety_method 'PSC', then saves
% codes/data_mpc/phi_comparison_results.mat (results_grid is 1×N).
% Figures: run plot_phi_comparison.m (PSC lateral-error overlay + exports).
%
% Saved variables: results_grid, safety_method_list, phi_list, EMAX, MU_VALUE
%   results_grid{1,k} — PSC result for phi_list(k,:)

clear; close all;

thisDir = fileparts(mfilename('fullpath'));   % .../codes/phi
codesDir = fileparts(thisDir);               % .../codes
addpath(thisDir, codesDir, ...
    fullfile(codesDir, 'impl_controller'), ...
    fullfile(codesDir, 'impl_model'), ...
    fullfile(codesDir, 'impl_estimator'), ...
    fullfile(codesDir, 'impl_road'));

DATA_FILE = fullfile(codesDir, 'data_mpc', 'phi_comparison_results.mat');

%% --- Safety method (PSC only; phi comparison is meaningful here) ---
safety_method_list = {'PSC'};

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
total_runs = N;
run_id = 0;
sm = 'PSC';

for k = 1:N
    run_id = run_id + 1;
    fprintf('\n=== Run %d/%d  PSC  phi=%s (%s) ===\n', ...
        run_id, total_runs, phi_list{k,2}, phi_list{k,1});

    res = main_single_run(phi_list{k,1}, EMAX, sm, MU_VALUE);

    res.name = phi_list{k,2};
    res.expr = phi_list{k,1};
    res.safety_method = sm;
    results_grid{1, k} = res;

    fprintf('  Completed: Max error = %.3f, Min prob = %.3f\n', ...
        max(abs(res.state.state(:,11))), min(res.prob.prob));
end

set_phi_expr('1 - (e/emax)^2');
disp('--- all runs completed ----')

d = fileparts(DATA_FILE);
if ~isempty(d) && ~isfolder(d)
    mkdir(d);
end
save(DATA_FILE, 'results_grid', 'phi_list', 'safety_method_list', 'EMAX', 'MU_VALUE', '-v7.3');
fprintf('Saved: %s\n', DATA_FILE);
fprintf('PSC figure: run(''phi/plot_phi_comparison.m'') from codes/ or open codes/phi/plot_phi_comparison.m.\n');
