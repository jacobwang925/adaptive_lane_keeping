% Compare different barrier function (phi) expressions side-by-side
%
% Calls main_single_run.m for each expression, collects results,
% and plots lane error, safety probability, and speed for comparison.
%
% IMPORTANT: The impact of phi expressions depends on the SAFETY_METHOD:
%   - 'DIRECT': Phi expressions MATTER. The MPC optimizer directly enforces
%     phi(e) >= 0, so the shape of phi affects the constraint gradient and
%     controller behavior. Use smooth, differentiable expressions only.
%   - 'PSC' (Proposed): Phi expressions have NO EFFECT in practice. The Monte
%     Carlo simulation only checks the sign of phi (phi < 0 = unsafe), and all
%     standard expressions cross zero at the same boundary (|e| = emax), so
%     p, LfP, LgP, and BP are identical regardless of phi shape.
%   - 'CDBF': Phi expressions are IGNORED. Uses control-dependent barrier
%     functions based on vehicle dynamics instead.
%   - 'NONE' (AMPC): Phi expressions are IGNORED. No safety constraints.
%
% NOTE on expression stability with DIRECT:
%   Non-smooth expressions (e.g. abs()) or those with steep gradients near the
%   boundary (e.g. cosine) can cause the gradient-based MPC solver to diverge.
%   Prefer smooth expressions like quadratic or quartic for stable results.

clear; close all;

%% --- Define phi expressions to compare ---
% Format: {'expression', 'display_name'; ...}
% The expression will be substituted into fun_safety_condition.m
%
% Available variables in expression:
%   e     - lateral error (state variable)
%   emax  - maximum allowed error (set by EMAX parameter)
%
% Example expressions (must be smooth and differentiable for DIRECT method):
%   '1 - (e/emax)^2'          - Quadratic (default), smooth, moderate gradient
%   '1 - (e/emax)^4'          - Quartic, flatter near center, steeper near boundary
%   '1 - (e/emax)^6'          - Sextic, even more forgiving near center
%   'cos(pi*e/(2*emax))'      - Cosine, smooth but steep gradient near boundary [may diverge]
%   '1 - abs(e/emax)'         - Linear, NON-DIFFERENTIABLE at e=0, avoid with DIRECT
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
SAFETY_METHODS = {'PSC', 'CDBF', 'DIRECT', 'NONE'};
MU_VALUE = 0.3;                    % Friction coefficient (0.3 = icy, 0.9 = dry)

M = numel(SAFETY_METHODS);

%% --- Plot styling constants ---
TICK_FS  = 9;
LABEL_FS = 10;
TITLE_FS = 11;

clr = [0.000, 0.447, 0.741;   % blue
       0.850, 0.325, 0.098;   % vermillion
       0.494, 0.184, 0.557;   % purple
       0.466, 0.674, 0.188];  % green

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

%% --- Run all safety methods and plot each ---
all_results = cell(M, 1);

for m = 1:M
    method = SAFETY_METHODS{m};
    fprintf('\n########## SAFETY METHOD: %s (%d/%d) ##########\n', method, m, M);

    results = cell(N, 1);
    for k = 1:N
        fprintf('\n=== Run %d/%d: %s  [phi = %s] ===\n', k, N, phi_list{k,2}, phi_list{k,1});

        res = main_single_run(phi_list{k,1}, EMAX, method, MU_VALUE);

        results{k} = res;
        results{k}.name = phi_list{k,2};
        results{k}.expr = phi_list{k,1};

        fprintf('  Completed: Max error = %.3f, Min prob = %.3f\n', ...
            max(abs(res.state.state(:,11))), min(res.prob.prob));
    end

    all_results{m} = results;

    %% --- Plot comparison for this method ---
    fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 18 18]);
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % (a) Lane error
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

    % (b) Safety probability
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

    % (c) Speed
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

    %% --- Save figure for this method ---
    fname = sprintf('data_mpc/figs_mpc/phi_comparison_%s', lower(method));
    exportgraphics(fig, [fname '.png'], 'Resolution', 600);
    exportgraphics(fig, [fname '.pdf'], 'ContentType', 'vector');
    fprintf('\nSaved to %s.{png,pdf}\n', fname);

    %% --- Print summary table for this method ---
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

% Restore default phi expression after all runs
set_phi_expr('1 - (e/emax)^2');
fprintf('\n--- All %d safety methods completed ---\n', M);
