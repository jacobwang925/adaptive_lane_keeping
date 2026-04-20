% Plot phi comparison from codes/data_mpc/phi_comparison_results.mat (no Simulink).
% Intended for PSC-only grids from compare_phi_expressions.m (results_grid is 1×N).
% Older .mat files with multiple methods (M×N) are still supported if PSC is listed.
%
% Single-panel figure — lateral error; all phi curves overlaid. Legend eastoutside.

clear; close all;

thisDir = fileparts(mfilename('fullpath'));   % .../codes/phi
codesDir = fileparts(thisDir);               % .../codes
addpath(thisDir, codesDir);
DATA_FILE = fullfile(codesDir, 'data_mpc', 'phi_comparison_results.mat');
exportDir = fullfile(codesDir, 'data_mpc', 'figs_mpc');
if ~isfolder(exportDir)
    mkdir(exportDir);
end

if ~isfile(DATA_FILE)
    error('Missing %s — run codes/phi/compare_phi_expressions.m first.', DATA_FILE);
end

S = load(DATA_FILE);
if isfield(S, 'results_grid') && isfield(S, 'safety_method_list')
    results_grid = S.results_grid;
    safety_method_list = normalizeMethodList(S.safety_method_list);
elseif isfield(S, 'results') && isfield(S, 'SAFETY_METHOD')
    results_grid = reshape(S.results, 1, numel(S.results));
    safety_method_list = normalizeMethodList({S.SAFETY_METHOD});
else
    error('Unrecognized format in %s', DATA_FILE);
end
phi_list = S.phi_list;

M = numel(safety_method_list);
N = size(phi_list, 1);
if ~isequal(size(results_grid), [M, N])
    error('results_grid must be %d×%d (methods × phi); got %s.', ...
        M, N, mat2str(size(results_grid)));
end

if M == 1
    idx_psc = 1;
else
    idx_psc = find(cellfun(@(s) strcmpi(char(string(s)), 'PSC'), safety_method_list), 1);
    if isempty(idx_psc)
        error('PSC not found in safety_method_list — cannot build PSC overlap figure.');
    end
end

TICK_FS  = 9;
LABEL_FS = 10;

% Blue, orange, yellow (gold for contrast on white), purple
clr = [0.00, 0.45, 0.75;
       0.95, 0.45, 0.10;
       0.90, 0.72, 0.12;
       0.55, 0.20, 0.65];

line_styles   = {'-', '--', '-.', ':'};
markers       = {'o', 's', 'd', '^'};
mark_every    = 50;
marker_offset = 12;

% Short names in legend (full expressions are in compare_phi_expressions / text)
leg_labels = cell(N, 1);
for k = 1:N
    short = char(string(phi_list{k,2}));
    short = regexprep(short, '\s*\(default\)\s*', '', 'ignorecase');
    short = strtrim(short);
    leg_labels{k} = sprintf('{\\it{\\phi}}: %s', short);
end

% TeX (not LaTeX): use {\it{e}} not {\it e} — otherwise \i is parsed as dotless-i and breaks \it.
ax_style = @(ax) set(ax, 'FontSize', TICK_FS, 'TickLabelInterpreter', 'none', ...
    'TickDir', 'out', 'GridAlpha', 0.15, 'Box', 'on', 'LineWidth', 0.5);

% Single axes; leave figure width for eastoutside legend (one column)
figW_cm = 15;
figH_cm = 7.5;
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 figW_cm figH_cm], ...
    'PaperUnits', 'centimeters', 'PaperPosition', [0 0 figW_cm figH_cm]);
set(fig, 'Units', 'normalized');

lm = 0.14;
tm = 0.07;
bm = 0.12;
plotW = 0.54;
plotY = bm + 0.02;
plotH = 1 - tm - plotY - 0.02;

ax1 = axes('Parent', fig, 'Position', [lm, plotY, plotW, plotH]); hold(ax1, 'on'); grid(ax1, 'on');
h_line = gobjects(N, 1);
tMax = 0;
for k = 1:N
    res = results_grid{idx_psc, k};
    t = timeValuesAsSeconds(res.state.Time);
    tMax = max(tMax, t(end));
    midx = 1 + (k-1)*marker_offset : mark_every : numel(t);
    mk = markers{mod(k-1,numel(markers))+1};
    c = clr(mod(k-1,size(clr,1))+1,:);
    h_line(k) = plot(ax1, t, res.state.state(:,11), ...
        line_styles{mod(k-1,numel(line_styles))+1}, ...
        'Color', c, 'LineWidth', 1.35, ...
        'Marker', mk, 'MarkerIndices', midx, 'MarkerSize', 5, ...
        'MarkerFaceColor', c, 'MarkerEdgeColor', min(c + 0.15, 1));
end
xlim(ax1, [0, ceil(tMax)]);
ylabel(ax1, 'Lateral error {\it{e}} (m)', 'Interpreter', 'tex', 'FontSize', LABEL_FS);
xlabel(ax1, 'Time (s)', 'Interpreter', 'none', 'FontSize', LABEL_FS);
ax_style(ax1);
if isfield(S, 'EMAX') && isfield(S, 'MU_VALUE')
    title(ax1, sprintf('PSC lateral error (e_{max}=%g m, \\mu=%.2f)', S.EMAX, S.MU_VALUE), ...
        'Interpreter', 'tex', 'FontSize', LABEL_FS);
end

lg = legend(ax1, h_line, leg_labels, 'Location', 'eastoutside', ...
    'Interpreter', 'tex', 'FontSize', TICK_FS, 'Box', 'off', 'NumColumns', 1);
lg.ItemTokenSize = [14, 10];

drawnow;

pngPath = fullfile(exportDir, 'fig_phi_comparison.png');
pdfPath = fullfile(exportDir, 'fig_phi_comparison.pdf');
exportgraphics(fig, pngPath, 'Resolution', 600);
exportgraphics(fig, pdfPath, 'ContentType', 'vector');
fprintf('\nSaved to %s and %s\n', pngPath, pdfPath);

printSummaryTable(results_grid, safety_method_list, phi_list);

%% --- local helpers ---

function sml = normalizeMethodList(sml)
    if isempty(sml)
        sml = {};
        return
    end
    if ~iscell(sml)
        sml = cellstr(sml);
    end
    sml = sml(:);
    for i = 1:numel(sml)
        sml{i} = char(string(sml{i}));
    end
end

function t = timeValuesAsSeconds(tvec)
    % Duration/datetime row times make MATLAB add a duplicate axis unit (e.g. "sec").
    if isduration(tvec)
        t = seconds(tvec);
    elseif isdatetime(tvec)
        t = seconds(tvec - tvec(1));
    else
        t = tvec;
    end
end

function printSummaryTable(results_grid, safety_method_list, phi_list)
    M = size(results_grid, 1);
    N = size(results_grid, 2);
    fprintf('\n');
    for m = 1:M
        sm = char(string(safety_method_list{m}));
        fprintf('--- %s ---\n', sm);
        fprintf('%-22s %10s %10s %10s %10s\n', 'Expression', 'MaxError', 'MeanError', 'MinProb', 'MeanProb');
        fprintf('%s\n', repmat('-', 1, 66));
        for k = 1:N
            res = results_grid{m, k};
            e_vals = res.state.state(:,11);
            p_vals = res.prob.prob;
            name = res.name;
            if isempty(name)
                name = phi_list{k,2};
            end
            fprintf('%-22s %10.3f %10.3f %10.3f %10.3f\n', ...
                char(string(name)), max(abs(e_vals)), mean(abs(e_vals)), min(p_vals), mean(p_vals));
        end
        fprintf('\n');
    end
end
