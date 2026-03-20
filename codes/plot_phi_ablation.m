%% plot_phi_ablation.m
% Publication-quality figure for LLM phi expression ablation results.
% Run AFTER run_llm_phi_ablation.m has completed and saved phi_ablation.mat.

clear; clc; close all;

%% 1. Load Data
load('../LLM/llm_results/phi_ablation.mat', 'all_results');

models      = ["gpt-4o-mini", "gemini-2.5-flash", "deepseek-chat"];
modelFields = ["gpt_4o_mini", "gemini_2_5_flash", "deepseek_chat"];
modelLabels = ["GPT-4o-mini", "Gemini-2.5-Flash", "DeepSeek-Chat"];

num_models = length(models);

%% 2. Compute Statistics
valid_rates   = zeros(1, num_models);
unique_counts = zeros(1, num_models);
total_valid   = zeros(1, num_models);
total_invalid = zeros(1, num_models);
category_data = cell(1, num_models);

for i = 1:num_models
    dat = all_results.(modelFields(i));
    n = length(dat.is_valid);

    valid_rates(i)   = dat.valid_rate * 100;
    total_valid(i)   = sum(dat.is_valid);
    total_invalid(i) = n - total_valid(i);

    valid_exprs = dat.expressions(dat.is_valid);
    if ~isempty(valid_exprs)
        [uniq, ~, idx] = unique(valid_exprs);
        counts = accumarray(idx, 1);
        unique_counts(i) = length(uniq);

        families = strings(length(uniq), 1);
        for j = 1:length(uniq)
            expr = lower(char(uniq(j)));
            if contains(expr, 'exp') && ~contains(expr, 'tanh') && ~contains(expr, 'cosh') && ~contains(expr, 'sech')
                families(j) = "Exponential";
            elseif contains(expr, 'tanh')
                families(j) = "Hyperbolic (tanh)";
            elseif contains(expr, 'cosh') || contains(expr, 'sech')
                families(j) = "Hyperbolic (cosh/sech)";
            elseif contains(expr, 'log')
                families(j) = "Logarithmic";
            elseif contains(expr, 'atan') || contains(expr, 'asin')
                families(j) = "Inverse Trig";
            elseif contains(expr, 'cos') || contains(expr, 'sin')
                families(j) = "Trigonometric";
            elseif contains(expr, 'abs')
                families(j) = "Absolute Value";
            else
                families(j) = "Polynomial";
            end
        end

        [fam_names, ~, fam_idx] = unique(families);
        fam_counts = zeros(length(fam_names), 1);
        for j = 1:length(fam_names)
            fam_counts(j) = sum(counts(fam_idx == j));
        end
        [fam_counts, sortI] = sort(fam_counts, 'descend');
        fam_names = fam_names(sortI);
        category_data{i} = table(fam_names, fam_counts, 'VariableNames', {'Family', 'Count'});
    else
        unique_counts(i) = 0;
        category_data{i} = table(strings(0,1), zeros(0,1), 'VariableNames', {'Family', 'Count'});
    end
end

%% 3. Color Palette (colorblind-friendly, muted academic tones)
% Model colors — distinct but not garish
clr_model = [0.122, 0.467, 0.706;   % steel blue   (GPT)
             0.839, 0.153, 0.157;   % muted red    (Gemini)
             0.173, 0.627, 0.173];  % forest green (DeepSeek)

% Family colors — qualitative Set2-inspired palette (max 8)
clr_fam = [0.400, 0.761, 0.647;   % teal
           0.988, 0.553, 0.384;   % salmon
           0.553, 0.627, 0.796;   % periwinkle
           0.906, 0.541, 0.765;   % mauve
           0.651, 0.847, 0.329;   % lime
           1.000, 0.851, 0.184;   % gold
           0.898, 0.769, 0.580;   % tan
           0.702, 0.702, 0.702];  % grey

%% 4. Figure Setup
fig = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 18 16]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

TICK_FS  = 9;
LABEL_FS = 10;
TITLE_FS = 11;
ANNOT_FS = 9;

%% Panel (a): Validity Rate
ax1 = nexttile(tl);
b1 = bar(valid_rates, 0.55, 'FaceColor', 'flat', 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.4);
for i = 1:num_models
    b1.CData(i,:) = clr_model(i,:);
end
set(ax1, 'XTickLabel', modelLabels, 'FontSize', TICK_FS, ...
    'TickLabelInterpreter', 'latex', 'TickDir', 'out', 'XTickLabelRotation', 20);
ylabel('Validity Rate (\%)', 'Interpreter', 'latex', 'FontSize', LABEL_FS);
title('(a) Expression Validity', 'Interpreter', 'latex', 'FontSize', TITLE_FS);
ylim([0 112]);
set(ax1, 'YGrid', 'on', 'XGrid', 'off', 'GridAlpha', 0.15, 'Box', 'on', 'LineWidth', 0.5);
for i = 1:num_models
    text(i, valid_rates(i) + 3, sprintf('%d\\%%', total_valid(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', ANNOT_FS, ...
        'Interpreter', 'latex', 'FontWeight', 'bold');
end

%% Panel (b): Unique Expressions
ax2 = nexttile(tl);
b2 = bar(unique_counts, 0.55, 'FaceColor', 'flat', 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.4);
for i = 1:num_models
    b2.CData(i,:) = clr_model(i,:);
end
set(ax2, 'XTickLabel', modelLabels, 'FontSize', TICK_FS, ...
    'TickLabelInterpreter', 'latex', 'TickDir', 'out', 'XTickLabelRotation', 20);
ylabel('Unique Expressions', 'Interpreter', 'latex', 'FontSize', LABEL_FS);
title('(b) Expression Diversity', 'Interpreter', 'latex', 'FontSize', TITLE_FS);
ylim([0, max(unique_counts)*1.2]);
set(ax2, 'YGrid', 'on', 'XGrid', 'off', 'GridAlpha', 0.15, 'Box', 'on', 'LineWidth', 0.5);
for i = 1:num_models
    text(i, unique_counts(i) + max(unique_counts)*0.03, sprintf('%d', unique_counts(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', ANNOT_FS, ...
        'Interpreter', 'latex', 'FontWeight', 'bold');
end

%% Panel (c): Expression Family Breakdown (stacked horizontal bar)
ax3 = nexttile(tl, [1, 2]);

all_families = [];
for i = 1:num_models
    if ~isempty(category_data{i})
        all_families = [all_families; category_data{i}.Family]; %#ok<AGROW>
    end
end
all_families = unique(all_families);
nf = length(all_families);

family_matrix = zeros(num_models, nf);
for i = 1:num_models
    cd = category_data{i};
    for j = 1:nf
        idx = find(cd.Family == all_families(j));
        if ~isempty(idx)
            family_matrix(i,j) = cd.Count(idx);
        end
    end
end

bh = barh(family_matrix, 'stacked', 'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 0.3);
for j = 1:nf
    bh(j).FaceColor = clr_fam(mod(j-1, size(clr_fam,1))+1, :);
end
set(ax3, 'YTickLabel', modelLabels, 'FontSize', TICK_FS, ...
    'TickLabelInterpreter', 'latex', 'TickDir', 'out');
xlabel('Number of Valid Expressions', 'Interpreter', 'latex', 'FontSize', LABEL_FS);
title('(c) Expression Family Breakdown', 'Interpreter', 'latex', 'FontSize', TITLE_FS);
set(ax3, 'XGrid', 'on', 'YGrid', 'off', 'GridAlpha', 0.15, 'Box', 'on', 'LineWidth', 0.5);

% Segment count labels inside bars
for i = 1:num_models
    cumx = 0;
    for j = 1:nf
        val = family_matrix(i, j);
        if val >= 4
            text(cumx + val/2, i, sprintf('%d', val), ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                'FontSize', 7, 'Interpreter', 'latex', 'Color', 'k');
        end
        cumx = cumx + val;
    end
end

lg = legend(all_families, 'Location', 'eastoutside', 'Interpreter', 'latex', ...
    'FontSize', 8, 'Box', 'off', 'NumColumns', 1);
lg.ItemTokenSize = [12, 12];

%% 5. Super Title
title(tl, '\textbf{LLM Barrier Function ($\phi$) Generation}\ ($N{=}100$ per model)', ...
    'Interpreter', 'latex', 'FontSize', 12);

%% 6. Export
exportgraphics(fig, 'data_mpc/figs_mpc/phi_ablation_academic.png', 'Resolution', 600);
exportgraphics(fig, 'data_mpc/figs_mpc/phi_ablation_academic.pdf', 'ContentType', 'vector');
fprintf('Saved to data_mpc/figs_mpc/phi_ablation_academic.{png,pdf}\n');

%% 7. Summary Table
fprintf('\n=== Summary ===\n');
fprintf('%-20s | %6s | %6s | %6s\n', 'Model', 'Valid', 'Unique', 'Rate');
fprintf('%s\n', repmat('-', 1, 50));
for i = 1:num_models
    fprintf('%-20s | %4d   | %4d   | %5.1f%%\n', modelLabels(i), total_valid(i), unique_counts(i), valid_rates(i));
end
