%% run_llm_pipeline.m
% End-to-end LLM-integrated lane keeping pipeline.
%   1. User provides a natural-language driving preference
%   2. LLM infers controller parameters + phi expression
%   3. Phi expression is validated; fallback to quadratic if invalid
%   4. Simulink simulation runs via main_single_run
%   5. Feedback is built from results
%   6. User provides follow-up preference
%   7. LLM reasons over feedback + new preference -> updated parameters
%   8. Second simulation runs, results are compared and saved

clear; clc; close all;

%% 1. Load API Keys from .env
env_str = fileread('../.env');
env_lines = splitlines(env_str);
for i = 1:length(env_lines)
    line = strtrim(env_lines{i});
    if ~isempty(line) && ~startsWith(line, '#') && contains(line, '=')
        idx = strfind(line, '=');
        key = strtrim(line(1:idx(1)-1));
        val = strtrim(line(idx(1)+1:end));
        setenv(key, val);
    end
end

%% 2. Configuration
LLM_MODEL     = "gemini-2.5-flash";
SAFETY_METHOD = "PSC";
MU_VALUE      = 0.3;
DEFAULT_PHI   = "1 - (e/emax)^2";
MAX_RETRIES   = 3;
RETRY_DELAY   = 2;

%% 3. User Inputs
user_input_1 = "I want to drive carefully, the road seems icy and I'm not confident about the conditions.";
user_input_2 = "That was too slow and cautious, I want to go faster. The road is actually fine.";

%% 4. Run 1 — Initial Inference
fprintf('\n========== RUN 1: Initial Inference ==========\n');
fprintf('User: "%s"\n\n', user_input_1);

S1 = call_llm_with_retry(@() llm_inferring(user_input_1, 'ModelName', LLM_MODEL), ...
    MAX_RETRIES, RETRY_DELAY);

phi_expr_1 = validate_phi(S1, DEFAULT_PHI);

fprintf('LLM parameters:\n');
fprintf('  e_max     = %g\n', S1.e_max);
fprintf('  mu_0      = %.2f\n', S1.mu_0);
fprintf('  sigma_0   = %.2f\n', S1.sigma_0);
fprintf('  bar_sigma = %.2f\n', S1.bar_sigma);
fprintf('  phi_expr  = %s\n', phi_expr_1);

fprintf('\nRunning simulation...\n');
results_1 = main_single_run(char(phi_expr_1), S1.e_max, char(SAFETY_METHOD), MU_VALUE);

%% 5. Build Feedback from Run 1
e1      = results_1.state.state(:,11);
speed1  = results_1.state.state(:,1);
prob1   = results_1.prob.prob;

nanidx = find(isnan(e1), 1);
if ~isempty(nanidx)
    e1     = e1(1:nanidx-1);
    speed1 = speed1(1:nanidx-1);
    prob1  = prob1(1:nanidx-1);
end

data_fb = sprintf([ ...
    'Run 1 summary:\n' ...
    '- Lane error 99.5%% quantile = %.3f m, max = %.3f m, mean = %.3f m\n' ...
    '- e_max was set to %g m\n' ...
    '- Average safety probability = %.2f (minimum = %.2f)\n' ...
    '- Maximum speed = %.2f m/s, mean speed = %.2f m/s\n' ...
    '- Barrier function used: %s\n' ...
    '- Duration: %d steps\n'], ...
    quantile(abs(e1), 0.995), max(abs(e1)), mean(abs(e1)), ...
    S1.e_max, mean(prob1), min(prob1), ...
    max(speed1), mean(speed1), phi_expr_1, numel(e1));

fprintf('\n--- Feedback to LLM ---\n%s\n', data_fb);

%% 6. Run 2 — Reasoning with Feedback
fprintf('\n========== RUN 2: Reasoning with Feedback ==========\n');
fprintf('User: "%s"\n\n', user_input_2);

S2 = call_llm_with_retry(@() llm_reasoning(data_fb, user_input_2, 'ModelName', LLM_MODEL), ...
    MAX_RETRIES, RETRY_DELAY);

phi_expr_2 = validate_phi(S2, DEFAULT_PHI);

fprintf('LLM parameters (updated):\n');
fprintf('  e_max     = %g\n', S2.e_max);
fprintf('  mu_0      = %.2f\n', S2.mu_0);
fprintf('  sigma_0   = %.2f\n', S2.sigma_0);
fprintf('  bar_sigma = %.2f\n', S2.bar_sigma);
fprintf('  phi_expr  = %s\n', phi_expr_2);

fprintf('\nRunning simulation...\n');
results_2 = main_single_run(char(phi_expr_2), S2.e_max, char(SAFETY_METHOD), MU_VALUE);

%% 7. Plot Comparison
e2     = results_2.state.state(:,11);
speed2 = results_2.state.state(:,1);
prob2  = results_2.prob.prob;

fig = figure('Color', 'w', 'Units', 'inches', 'Position', [2 2 12 10]);
run_labels = {"Run 1 (initial)", "Run 2 (after feedback)"};
clr = [0.220 0.557 0.784; 0.851 0.325 0.310];

% Lane error
subplot(3,1,1); hold on; grid on; box on;
plot(results_1.state.Time, results_1.state.state(:,11), '-', 'Color', clr(1,:), 'LineWidth', 1.5);
plot(results_2.state.Time, results_2.state.state(:,11), '--', 'Color', clr(2,:), 'LineWidth', 1.5);
ylabel('Lane Error $e$ [m]', 'Interpreter', 'latex', 'FontSize', 12);
title('LLM-Integrated Pipeline: Run Comparison', 'Interpreter', 'latex', 'FontSize', 13);
legend(run_labels, 'Interpreter', 'latex', 'Location', 'best', 'Box', 'off');
set(gca, 'FontSize', 11, 'TickLabelInterpreter', 'latex');

% Safety probability
subplot(3,1,2); hold on; grid on; box on;
plot(results_1.prob.Time, prob1, '-', 'Color', clr(1,:), 'LineWidth', 1.5);
plot(results_2.prob.Time, prob2, '--', 'Color', clr(2,:), 'LineWidth', 1.5);
ylabel('Safety Probability $P$', 'Interpreter', 'latex', 'FontSize', 12);
legend(run_labels, 'Interpreter', 'latex', 'Location', 'best', 'Box', 'off');
set(gca, 'FontSize', 11, 'TickLabelInterpreter', 'latex');

% Speed
subplot(3,1,3); hold on; grid on; box on;
plot(results_1.state.Time, results_1.state.state(:,1)*3.6, '-', 'Color', clr(1,:), 'LineWidth', 1.5);
plot(results_2.state.Time, results_2.state.state(:,1)*3.6, '--', 'Color', clr(2,:), 'LineWidth', 1.5);
ylabel('Speed [km/h]', 'Interpreter', 'latex', 'FontSize', 12);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 12);
legend(run_labels, 'Interpreter', 'latex', 'Location', 'best', 'Box', 'off');
set(gca, 'FontSize', 11, 'TickLabelInterpreter', 'latex');

%% 8. Summary Table
fprintf('\n=== Results Summary ===\n');
fprintf('%-20s %10s %10s %10s %10s %12s\n', '', 'MaxErr', 'MeanErr', 'MinProb', 'MeanProb', 'phi');
fprintf('%s\n', repmat('-', 1, 75));
for r = 1:2
    if r == 1
        ev = e1; pv = prob1; phi_str = phi_expr_1;
    else
        ev = e2; pv = prob2; phi_str = phi_expr_2;
    end
    fprintf('%-20s %10.3f %10.3f %10.3f %10.3f %s\n', ...
        run_labels{r}, max(abs(ev)), mean(abs(ev)), min(pv), mean(pv), phi_str);
end

%% 9. Save
savefile = sprintf('data_mpc/llm_pipeline_%s_%s.mat', ...
    strrep(strrep(char(LLM_MODEL), '-', '_'), '.', '_'), ...
    char(SAFETY_METHOD));
save(savefile, 'results_1', 'results_2', 'S1', 'S2', ...
    'user_input_1', 'user_input_2', 'data_fb', ...
    'phi_expr_1', 'phi_expr_2', 'LLM_MODEL', 'SAFETY_METHOD', 'MU_VALUE');
fprintf('\nSaved to %s\n', savefile);

exportgraphics(fig, 'data_mpc/figs_mpc/llm_pipeline_comparison.png', 'Resolution', 300);
fprintf('Figure saved to data_mpc/figs_mpc/llm_pipeline_comparison.png\n');

% Restore default phi
set_phi_expr('1 - (e/emax)^2');

%% ===== Local Functions =====

function S = call_llm_with_retry(llm_fn, max_retries, delay)
    for attempt = 1:max_retries
        try
            S = llm_fn();
            fprintf('LLM call succeeded (attempt %d/%d)\n', attempt, max_retries);
            return;
        catch ME
            fprintf('LLM attempt %d/%d failed: %s\n', attempt, max_retries, ME.message);
            if attempt < max_retries
                pause(delay);
            else
                rethrow(ME);
            end
        end
    end
end

function phi_expr = validate_phi(S, default_phi)
    if ~isfield(S, 'phi_expr') || isempty(S.phi_expr)
        fprintf('No phi_expr returned, using default: %s\n', default_phi);
        phi_expr = default_phi;
        return;
    end

    raw = string(S.phi_expr);
    clean = strrep(strrep(strrep(raw, '.*', '*'), './', '/'), '.^', '^');
    try
        fn = str2func("@(e, emax) " + clean);
        v0 = double(fn(0, S.e_max));
        ve = double(fn(S.e_max, S.e_max));
        if isreal(v0) && isreal(ve) && v0 >= 0 && ve <= 0
            phi_expr = raw;
            fprintf('phi_expr validated: %s  [phi(0)=%.3f, phi(emax)=%.3f]\n', phi_expr, v0, ve);
            return;
        end
        fprintf('phi_expr failed bounds check (phi(0)=%.3f, phi(emax)=%.3f), using default\n', v0, ve);
    catch ME
        fprintf('phi_expr eval error (%s), using default\n', ME.message);
    end
    phi_expr = default_phi;
end
