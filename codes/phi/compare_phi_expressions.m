% Compare barrier function (phi) expressions across all safety methods
%
% Runs main_single_run for every pair (safety method × phi row), then saves
% codes/data_mpc/phi_comparison_results.mat and exports one 3-panel figure per method
% under codes/data_mpc/figs_mpc/phi_comparison_<method>.{png,pdf}. For PSC-only overlap
% plot, run plot_phi_comparison.m after loading the .mat.
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
fprintf('Saved: %s\n', DATA_FILE);

%% --- Per-method figures (lane error, safety probability, speed) ---
exportDir = fullfile(codesDir, 'data_mpc', 'figs_mpc');
if ~isfolder(exportDir)
    mkdir(exportDir);
end

TICK_FS  = 9;
LABEL_FS = 10;
TITLE_FS = 11;
clr = [0.000, 0.447, 0.741;
       0.850, 0.325, 0.098;
       0.494, 0.184, 0.557;
       0.466, 0.674, 0.188];
line_styles   = {'-', '--', '-.', ':'};
markers       = {'o', 's', 'd', '^'};
mark_every    = 50;
marker_offset = 12;

leg_labels = cell(N, 1);
for k = 1:N
    leg_labels{k} = sprintf('$\\phi = %s$ (%s)', phi_list{k,1}, phi_list{k,2});
end
ax_style = @(ax) set(ax, 'FontSize', TICK_FS, 'TickLabelInterpreter', 'latex', ...
    'TickDir', 'out', 'GridAlpha', 0.15, 'Box', 'on', 'LineWidth', 0.5);

for mi = 1:M
    method = char(safety_method_list{mi});
    results = cell(N, 1);
    for k = 1:N
        results{k} = results_grid{mi, k};
    end

    fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 18 18]);
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl); hold(ax1, 'on'); grid(ax1, 'on');
    for k = 1:N
        t = results{k}.state.Time;
        midx = 1 + (k-1)*marker_offset : mark_every : numel(t);
        plot(ax1, t, results{k}.state.state(:,11), ...
            line_styles{mod(k-1,numel(line_styles))+1}, ...
            'Color', clr(mod(k-1,size(clr,1))+1,:), 'LineWidth', 1.3, ...
            'Marker', markers{mod(k-1,numel(markers))+1}, ...
            'MarkerIndices', midx, 'MarkerSize', 4);
    end
    ylabel(ax1, 'Lateral Error $e$ [m]', 'Interpreter', 'latex', 'FontSize', LABEL_FS);
    title(ax1, sprintf('(a) Lane Tracking Error (%s)', method), ...
        'Interpreter', 'latex', 'FontSize', TITLE_FS);
    ax_style(ax1);
    set(ax1, 'XTickLabel', []);
    lg1 = legend(ax1, leg_labels, 'Interpreter', 'latex', 'Location', 'best', ...
        'FontSize', 8, 'Box', 'off');
    lg1.ItemTokenSize = [20, 8];

    ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on');
    for k = 1:N
        t = results{k}.prob.Time;
        midx = 1 + (k-1)*marker_offset : mark_every : numel(t);
        plot(ax2, t, results{k}.prob.prob, ...
            line_styles{mod(k-1,numel(line_styles))+1}, ...
            'Color', clr(mod(k-1,size(clr,1))+1,:), 'LineWidth', 1.3, ...
            'Marker', markers{mod(k-1,numel(markers))+1}, ...
            'MarkerIndices', midx, 'MarkerSize', 4);
    end
    ylabel(ax2, 'Safety Probability $P$', 'Interpreter', 'latex', 'FontSize', LABEL_FS);
    title(ax2, '(b) Safety Probability', 'Interpreter', 'latex', 'FontSize', TITLE_FS);
    ax_style(ax2);
    set(ax2, 'XTickLabel', []);

    ax3 = nexttile(tl); hold(ax3, 'on'); grid(ax3, 'on');
    for k = 1:N
        t = results{k}.state.Time;
        midx = 1 + (k-1)*marker_offset : mark_every : numel(t);
        plot(ax3, t, results{k}.state.state(:,1)*3.6, ...
            line_styles{mod(k-1,numel(line_styles))+1}, ...
            'Color', clr(mod(k-1,size(clr,1))+1,:), 'LineWidth', 1.3, ...
            'Marker', markers{mod(k-1,numel(markers))+1}, ...
            'MarkerIndices', midx, 'MarkerSize', 4);
    end
    ylabel(ax3, 'Speed [km/h]', 'Interpreter', 'latex', 'FontSize', LABEL_FS);
    xlabel(ax3, 'Time [s]', 'Interpreter', 'latex', 'FontSize', LABEL_FS);
    title(ax3, '(c) Longitudinal Speed', 'Interpreter', 'latex', 'FontSize', TITLE_FS);
    ax_style(ax3);

    title(tl, sprintf('\\textbf{Barrier Function Comparison}\\ ($e_{\\max}{=}%g$ m, $\\mu{=}%.1f$, %s)', ...
        EMAX, MU_VALUE, method), 'Interpreter', 'latex', 'FontSize', 12);
    linkaxes([ax1 ax2 ax3], 'x');

    fname = fullfile(exportDir, sprintf('phi_comparison_%s', lower(method)));
    exportgraphics(fig, [fname '.png'], 'Resolution', 600);
    exportgraphics(fig, [fname '.pdf'], 'ContentType', 'vector');
    fprintf('Saved figure: %s.{png,pdf}\n', fname);

    fprintf('\n--- Summary: %s ---\n', method);
    fprintf('%-25s %10s %10s %10s %10s\n', 'Expression', 'MaxError', 'MeanError', 'MinProb', 'MeanProb');
    fprintf('%s\n', repmat('-', 1, 70));
    for k = 1:N
        e_vals = results{k}.state.state(:,11);
        p_vals = results{k}.prob.prob;
        fprintf('%-25s %10.3f %10.3f %10.3f %10.3f\n', ...
            results{k}.name, max(abs(e_vals)), mean(abs(e_vals)), min(p_vals), mean(p_vals));
    end
end

fprintf('\nPSC overlap plot: run(''phi/plot_phi_comparison.m'') from codes/ or open codes/phi/plot_phi_comparison.m.\n');
