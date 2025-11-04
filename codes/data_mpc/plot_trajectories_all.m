%% plot_trajectories_all.m
% Batch visualization for MPC sweep (AMPC, CDBF, APSC)
% Matches files saved by run_closed_loop_parallel.m:
%   data_mpc/data_<CTRL>_multi_<condition>_H10_prior_... .mat
% where the param string starts with a leading underscore.

clc;

% ---------------- User knobs ----------------
baseDir         = ".";     % where the MAT files live
outRoot         = "figs_mpc";     % where to save figures
Hstr            = "H10";          % horizon string used in filenames
numToPlot       = 10;             % number of Monte Carlo trajectories per controller
saveEPS         = true;           % also save EPS alongside PNG
makeOverlay     = true;           % single-axes comparison
makeSideBySide  = true;           % 3-panel tiledlayout
makeSummaryCSV  = true;           % save simple metrics per case
% --------------------------------------------

% These lists must match the sweep used during simulation
prior_ic1_list = [0.3, 0.5, 0.9];
prior_ic2_list = [0.05, 0.3];
mes_var_list   = [0.05, 0.3];
emax_list      = [3, 5, 10];
mu_gt_list     = [0.3, 0.5, 0.8];

% Helpers
sanitizeNum = @(x) strrep(num2str(x), '.', 'p');  % 0.05 -> "0p05"
[condFromMu, ~] = makeCondMapper();               % discretize-based mapper

% Create output root
if ~isfolder(outRoot), mkdir(outRoot); end

summaryRows = {};
caseCount = 0;
fprintf('=== Scanning saved results & rendering figures ===\n');

for p1 = prior_ic1_list
for p2 = prior_ic2_list
for mv = mes_var_list
for em = emax_list
for mu = mu_gt_list

    condition = condFromMu(mu);  % "icy" | "normal" | "dry"
    p1s = sanitizeNum(p1);
    p2s = sanitizeNum(p2);
    mvs = sanitizeNum(mv);
    ems = sanitizeNum(em);

    % IMPORTANT: param_str has a leading underscore to match your saver
    param_str = sprintf("_prior_%s_%s_mesvar_%s_emax_%s", p1s, p2s, mvs, ems);

    % Filenames EXACTLY as your saver wrote them
    fAMPC = fullfile(baseDir, sprintf("data_AMPC_multi_%s_%s%s.mat", condition, Hstr, param_str));
    fCDBF = fullfile(baseDir, sprintf("data_CDBF_multi_%s_%s%s.mat", condition, Hstr, param_str));
    fAPSC = fullfile(baseDir, sprintf("data_APSC_multi_%s_%s%s.mat", condition, Hstr, param_str));

    if ~(isfile(fAMPC) && isfile(fCDBF) && isfile(fAPSC))
        fprintf('[skip] Missing files:\n  %s\n  %s\n  %s\n', fAMPC, fCDBF, fAPSC);
        continue;
    end

    % Load triplet
    try
        AMPC = load(fAMPC);
        CDBF = load(fCDBF);
        APSC = load(fAPSC);
    catch ME
        fprintf('[skip] Failed to load %s | %s: %s\n', condition, param_str, ME.message);
        continue;
    end

    % Destination folder
    outDir = fullfile(outRoot, condition, erase(param_str, "_")); % folder name without leading _
    if ~isfolder(outDir), mkdir(outDir); end

    % ---- FIGURE A: Overlay ----
    if makeOverlay
        try
            figA = figure('Visible','off','Color','w');
            clf(figA,'reset');
            figA.Position(3:4) = [700 600];
            hold on; box on; axis equal;

            % Legend exemplars
            i1 = 1;
            plot(rmmissing(APSC.TRAJ(i1,1:end-1,1)), rmmissing(APSC.TRAJ(i1,1:end-1,2)), ...
                'Color', [0 0.4470 0.7410], 'DisplayName','Proposed (APSC)', 'LineWidth', 1.2);
            plot(rmmissing(AMPC.TRAJ(i1,:,1)), rmmissing(AMPC.TRAJ(i1,:,2)), ...
                'Color', [0.8500 0.3250 0.0980], 'DisplayName','Adaptive MPC', 'LineWidth', 1.2);
            plot(rmmissing(CDBF.TRAJ(i1,1:end-1,1)), rmmissing(CDBF.TRAJ(i1,1:end-1,2)), ...
                'Color', [0.9290 0.6940 0.1250], 'DisplayName','CDBF', 'LineWidth', 1.2);

            % Additional trajectories
            nPlot = min([numToPlot, size(APSC.TRAJ,1), size(AMPC.TRAJ,1), size(CDBF.TRAJ,1)]);
            for k = 1:nPlot
                plot(rmmissing(APSC.TRAJ(k,1:end-1,1)), rmmissing(APSC.TRAJ(k,1:end-1,2)), ...
                    'Color', [0 0.4470 0.7410], 'HandleVisibility','off');
                plot(rmmissing(AMPC.TRAJ(k,:,1)), rmmissing(AMPC.TRAJ(k,:,2)), ...
                    'Color', [0.8500 0.3250 0.0980], 'HandleVisibility','off');
                plot(rmmissing(CDBF.TRAJ(k,1:end-1,1)), rmmissing(CDBF.TRAJ(k,1:end-1,2)), ...
                    'Color', [0.9290 0.6940 0.1250], 'HandleVisibility','off');
            end

            drawLaneBounds();

            xlabel('Distance (m)');
            ylabel('Distance (m)');
            title(sprintf('%s | %s', upper(condition), strrep(erase(param_str, "_"),'_',' ')));
            xlim([0, 90]); ylim([-20, 150]);
            legend('Location','northwest');
            hold off;

            baseName = fullfile(outDir, sprintf('traj_overlay_%s_%s', condition, erase(param_str, "_")));
            saveas(figA, baseName, 'png');
            if saveEPS, saveas(figA, baseName, 'epsc'); end
            close(figA);
        catch ME
            fprintf('[warn] Overlay failed for %s | %s: %s\n', condition, param_str, ME.message);
        end
    end

    % ---- FIGURE B: Side-by-side ----
    if makeSideBySide
        try
            figB = figure('Visible','off','Color','w');
            clf(figB,'reset');
            figB.Position(3:4) = [1200 400];
            tl = tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

            controllers = {'APSC','AMPC','CDBF'};
            dataCell    = {APSC,  AMPC,  CDBF};
            colors      = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.9290 0.6940 0.1250]};

            % Consistent n across controllers
            nPlot = min([numToPlot, size(APSC.TRAJ,1), size(AMPC.TRAJ,1), size(CDBF.TRAJ,1)]);

            for c = 1:3
                nexttile;
                hold on; axis equal; box on;
                for k = 1:nPlot
                    if controllers{c}=="AMPC"
                        tx = rmmissing(dataCell{c}.TRAJ(k,:,1));
                        ty = rmmissing(dataCell{c}.TRAJ(k,:,2));
                    else
                        tx = rmmissing(dataCell{c}.TRAJ(k,1:end-1,1));
                        ty = rmmissing(dataCell{c}.TRAJ(k,1:end-1,2));
                    end
                    plot(tx, ty, 'Color', colors{c});
                end
                drawLaneBounds();
                xlim([0, 90]); ylim([-20, 150]);
                title(controllers{c});
                if c==1, ylabel('Distance (m)'); else, set(gca,'YTickLabel',[]); end
                xlabel('Distance (m)');
                hold off;
            end

            title(tl, sprintf('%s | %s', upper(condition), strrep(erase(param_str, "_"),'_',' ')));

            baseName = fullfile(outDir, sprintf('traj_sideBySide_%s_%s', condition, erase(param_str, "_")));
            saveas(figB, baseName, 'png');
            if saveEPS, saveas(figB, baseName, 'epsc'); end
            close(figB);
        catch ME
            fprintf('[warn] Side-by-side failed for %s | %s: %s\n', condition, param_str, ME.message);
        end
    end

    % ---- Optional: Summary metrics per case ----
    if makeSummaryCSV
        try
            metrics = struct();
            metrics.APSC_minProb = row_min_ignore_nan(APSC.PROB);
            metrics.AMPC_minProb = row_min_ignore_nan(AMPC.PROB);
            metrics.CDBF_minProb = row_min_ignore_nan(CDBF.PROB);
            metrics.APSC_meanVx  = row_mean_ignore_nan(APSC.SPEED);
            metrics.AMPC_meanVx  = row_mean_ignore_nan(AMPC.SPEED);
            metrics.CDBF_meanVx  = row_mean_ignore_nan(CDBF.SPEED);

            summaryRows(end+1,:) = { ...
                string(condition), string(erase(param_str, "_")), ...
                mean(metrics.APSC_minProb,'omitnan'), mean(metrics.AMPC_minProb,'omitnan'), mean(metrics.CDBF_minProb,'omitnan'), ...
                mean(metrics.APSC_meanVx,'omitnan'),  mean(metrics.AMPC_meanVx,'omitnan'),  mean(metrics.CDBF_meanVx,'omitnan') ...
            };
        catch ME
            fprintf('[warn] Metrics failed for %s | %s: %s\n', condition, param_str, ME.message);
        end
    end

    caseCount = caseCount + 1;
    fprintf('[ok] %s | %s\n', condition, param_str);

end
end
end
end
end

fprintf('=== Done. Rendered %d cases. ===\n', caseCount);

% Dump summary CSV
if makeSummaryCSV && ~isempty(summaryRows)
    headers = ["condition","param_str", ...
               "APSC_minProb_mean","AMPC_minProb_mean","CDBF_minProb_mean", ...
               "APSC_meanVx","AMPC_meanVx","CDBF_meanVx"];
    T = cell2table(summaryRows, 'VariableNames', cellstr(headers));
    outCSV = fullfile(outRoot, "summary_metrics.csv");
    writetable(T, outCSV);
    fprintf('Saved summary CSV: %s\n', outCSV);
end


%% ================= Local helpers =================
function [condFromMu, labels] = makeCondMapper()
% Return a function handle mapping mu -> "icy" | "normal" | "dry"
    edges  = [-inf, 0.4, 0.7, inf];     % [-inf,0.4), [0.4,0.7], (0.7,inf)
    labels = ["icy","normal","dry"];
    condFromMu = @(mu) labels(discretize(mu, edges));
end

function drawLaneBounds()
% Same lane boundaries as in your single-case script
    t = 0:0.1:1;
    left_x  = [t*30, 30+35*cos(pi/2*(t-1)), t*0+65];
    left_y  = [t*0+5, 40+35*sin(pi/2*(t-1)), t*30+40];
    right_x = [t*30, 30+45*cos(pi/2*(t-1)), t*0+75];
    right_y = [t*0-5, 40+45*sin(pi/2*(t-1)), t*30+40];
    plot(left_x, left_y, 'k', 'HandleVisibility','off');
    plot(right_x, right_y, 'k', 'HandleVisibility','off');
end

function v = row_min_ignore_nan(M)
% Row-wise min ignoring NaNs (M: [num_sims x T])
    v = nan(size(M,1),1);
    for ii = 1:size(M,1)
        v(ii) = min(M(ii, :), [], 'omitnan');
    end
end

function v = row_mean_ignore_nan(M)
% Row-wise mean ignoring NaNs (M: [num_sims x T])
    v = nan(size(M,1),1);
    for ii = 1:size(M,1)
        v(ii) = mean(M(ii, :), 'omitnan');
    end
end
