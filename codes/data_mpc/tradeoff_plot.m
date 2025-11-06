%% tradeoff_safety_efficiency_clean.m
% Clean Safety–Efficiency tradeoff plot (filtered cases)
% - Keeps MATLAB default font
% - Removes "(Feasible Cases)" from title
% - Uses compact 5x4 inch figure

clc; close all;

csvPath = fullfile("figs_mpc", "summary_metrics.csv");
if ~isfile(csvPath)
    error("summary_metrics.csv not found at %s", csvPath);
end
T = readtable(csvPath);

colsProb = ["APSC_minProb_mean","AMPC_minProb_mean","CDBF_minProb_mean"];
colsVx   = ["APSC_meanVx","AMPC_meanVx","CDBF_meanVx"];


main_draft = false; % smaller plot in main draft to save space

threshold = 0.3; % filtering infeasible cases
% threshold = 0; % all cases

% --- Filter infeasible cases ---
mask_feasible = all(T{:, colsProb} >= threshold, 2);
num_infeasible = sum(~mask_feasible);
T = T(mask_feasible,:);
fprintf('Removed %d infeasible cases (safety prob < 0.3).\n', num_infeasible);
fprintf('Remaining %d feasible cases.\n', height(T));
if height(T) == 0
    warning('No feasible cases left after filtering.'); return;
end

% --- Prepare stats ---
controllers = {'APSC','AMPC','CDBF'};
colors = [0 0.4470 0.7410;
          0.8500 0.3250 0.0980;
          0.9290 0.6940 0.1250];
stats = struct();

for i = 1:3
    vx = T.(colsVx(i));
    pr = T.(colsProb(i));
    vx = vx(~isnan(vx) & ~isnan(pr));
    pr = pr(~isnan(vx) & ~isnan(pr));
    stats.(controllers{i}).vx_mean = mean(vx);
    stats.(controllers{i}).pr_mean = mean(pr);
    stats.(controllers{i}).cov     = cov(vx, pr);
end

% --- Plot ---

if main_draft
    f = figure('Color','w','Units','inches','Position',[2 2 5 3]);
else
    f = figure('Color','w','Units','inches','Position',[2 2 5 4]);
end
hold on; grid on; box on;
set(gca,'FontSize',12);

for i = 1:3
    c = controllers{i};
    col = colors(i,:);
    vx = T.(colsVx(i));
    pr = T.(colsProb(i));

    scatter(vx, pr, 25, col, 'filled', 'MarkerFaceAlpha',0.3, ...
        'DisplayName', sprintf('%s samples', c));

    mu = [stats.(c).vx_mean, stats.(c).pr_mean];
    plot(mu(1), mu(2), 'o', 'MarkerFaceColor', col, 'MarkerEdgeColor','k', ...
         'MarkerSize',6, 'DisplayName', sprintf('%s mean', c));

    plotErrorEllipse(mu, stats.(c).cov, col, 1, sprintf('%s variance', c));
end

xlabel('Average Speed (m/s)', 'FontSize',12);
ylabel('Minimum Safety Probability', 'FontSize',12);
% title('Safety–Efficiency Tradeoff', 'FontSize',13);

legend({'APSC samples','APSC mean','APSC variance', ...
        'AMPC samples','AMPC mean','AMPC variance', ...
        'CDBF samples','CDBF mean','CDBF variance'}, ...
        'Location','southwest','FontSize',10,'Box','on');

if threshold == 0
    xlim([4, 9]);
    ylim([0 1]); 
    saveas(f,'tradeoff_safety_efficiency_all.png');
    saveas(f,'tradeoff_safety_efficiency_all','epsc')
    fprintf('Saved: tradeoff_safety_efficiency_all.png\n');
elseif threshold == 0.3
    if main_draft
        ylim([0.5 1]);
        saveas(f,'tradeoff_safety_efficiency_main.png');
        saveas(f,'tradeoff_safety_efficiency_main','epsc')
        fprintf('Saved: tradeoff_safety_efficiency_main.png\n');
    else
        ylim([0.3 1]); 
        axis tight;
        saveas(f,'tradeoff_safety_efficiency.png');
        saveas(f,'tradeoff_safety_efficiency','epsc')
        fprintf('Saved: tradeoff_safety_efficiency.png\n');
    end
end

%% --- Helper for covariance ellipse ---
function plotErrorEllipse(mu, Sigma, color, nsig, name)
    if nargin < 4, nsig = 1; end
    if nargin < 5, name = ''; end
    if any(isnan(Sigma(:))) || rank(Sigma) < 2, return; end
    [V,D] = eig(Sigma);
    if any(diag(D)<=0), return; end
    t = linspace(0,2*pi,100);
    a = nsig * sqrt(D(1,1)); b = nsig * sqrt(D(2,2));
    xy = V * [a*cos(t); b*sin(t)];
    plot(xy(1,:)+mu(1), xy(2,:)+mu(2), 'Color', color, 'LineWidth',1.3, 'DisplayName', name);
end
