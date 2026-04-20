%% run_llm_pipeline.m
% End-to-end LLM-integrated lane keeping pipeline.
%   1. User provides a natural-language driving preference
%   2. LLM infers controller parameters + phi expression
%   3. Phi expression is validated; fallback to quadratic if invalid
%   4. Simulink simulation runs via main_single_run
%   5. Feedback is built from Run 1
%   6. User follow-up -> LLM reasoning -> second simulation (Run 2)
%   7. Cumulative feedback (Runs 1--2) + third user message -> LLM -> Run 3
%   8. Save .mat with all instructions, phi, trajectories (no figures)

clear; clc; close all;

codesDir = fileparts(fileparts(mfilename('fullpath')));  % .../codes (script lives in codes/phi/)
repoRoot = fileparts(codesDir);              % repository root

% Simulink + helpers assume cwd is codes/ (main_single_run uses relative addpath).
userDir = pwd;
cd(codesDir);
cleanupObj = onCleanup(@() cd(userDir)); %#ok<NASGU>

addpath(fullfile(codesDir, 'phi'));

%% 1. Load API Keys from .env
env_str = fileread(fullfile(repoRoot, '.env'));
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
% Narrative: Run 1 = conservative; Run 2 = faster / more aggressive; Run 3 = middle ground (comfort vs pace).
user_input_1 = "I want to drive carefully, the road seems icy and I'm not confident about the conditions. Prioritize staying well inside the lane even if that feels slow.";
user_input_2 = "That was too slow and cautious for me. I want to go noticeably faster and be more aggressive with lane tolerance—the road is dry and I'm confident. Please push toward a more performance-oriented setup than before.";
user_input_3 = "Find a balance between comfort and getting there quickly: not as timid as the first run, but not as aggressive as the second. I want a reasonable middle ground.";

%% 4. Run 1 — Initial Inference
fprintf('\n========== RUN 1: Initial Inference ==========\n');
fprintf('User: "%s"\n\n', user_input_1);

S1 = call_llm_with_retry(@() llm_inferring(user_input_1, 'ModelName', LLM_MODEL), ...
    MAX_RETRIES, RETRY_DELAY);

phi_llm_returned_1 = llmPhiFromStruct(S1);
[phi_expr_1, phi_llm_validated_1] = validate_phi(S1, DEFAULT_PHI);

fprintf('LLM parameters:\n');
fprintf('  e_max     = %g\n', S1.e_max);
fprintf('  mu_0      = %.2f\n', S1.mu_0);
fprintf('  sigma_0   = %.2f\n', S1.sigma_0);
fprintf('  bar_sigma = %.2f\n', S1.bar_sigma);
fprintf('  init_v    = %g m/s\n', S1.init_v);
fprintf('  phi from LLM (JSON):        %s\n', charOrPlaceholder(phi_llm_returned_1));
fprintf('  phi used in simulation:     %s\n', char(phi_expr_1));

opts1 = struct('mu_0', S1.mu_0, 'sigma_0', S1.sigma_0, ...
    'bar_sigma', S1.bar_sigma, 'init_v', S1.init_v);
fprintf('\nRunning simulation...\n');
results_1 = main_single_run(char(phi_expr_1), S1.e_max, char(SAFETY_METHOD), MU_VALUE, opts1);

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

phi_llm_returned_2 = llmPhiFromStruct(S2);
[phi_expr_2, phi_llm_validated_2] = validate_phi(S2, DEFAULT_PHI);

fprintf('LLM parameters (updated):\n');
fprintf('  e_max     = %g\n', S2.e_max);
fprintf('  mu_0      = %.2f\n', S2.mu_0);
fprintf('  sigma_0   = %.2f\n', S2.sigma_0);
fprintf('  bar_sigma = %.2f\n', S2.bar_sigma);
fprintf('  init_v    = %g m/s\n', S2.init_v);
fprintf('  phi from LLM (JSON):        %s\n', charOrPlaceholder(phi_llm_returned_2));
fprintf('  phi used in simulation:     %s\n', char(phi_expr_2));

opts2 = struct('mu_0', S2.mu_0, 'sigma_0', S2.sigma_0, ...
    'bar_sigma', S2.bar_sigma, 'init_v', S2.init_v);
fprintf('\nRunning simulation...\n');
results_2 = main_single_run(char(phi_expr_2), S2.e_max, char(SAFETY_METHOD), MU_VALUE, opts2);

%% 6b. Build cumulative feedback (Run 1 + Run 2) for third LLM call
e2     = results_2.state.state(:,11);
speed2 = results_2.state.state(:,1);
prob2  = results_2.prob.prob;

nanidx2 = find(isnan(e2), 1);
if ~isempty(nanidx2)
    e2     = e2(1:nanidx2-1);
    speed2 = speed2(1:nanidx2-1);
    prob2  = prob2(1:nanidx2-1);
end

data_fb_2 = sprintf([ ...
    'Run 2 summary:\n' ...
    '- Lane error 99.5%% quantile = %.3f m, max = %.3f m, mean = %.3f m\n' ...
    '- e_max was set to %g m\n' ...
    '- Average safety probability = %.2f (minimum = %.2f)\n' ...
    '- Maximum speed = %.2f m/s, mean speed = %.2f m/s\n' ...
    '- Barrier function used: %s\n' ...
    '- Duration: %d steps\n'], ...
    quantile(abs(e2), 0.995), max(abs(e2)), mean(abs(e2)), ...
    S2.e_max, mean(prob2), min(prob2), ...
    max(speed2), mean(speed2), phi_expr_2, numel(e2));

% Run 3 reasoning: cumulative quantitative feedback only (same pattern as Run 2 seeing Run 1).
data_fb_cum = sprintf('%s\n%s', data_fb, data_fb_2);
fprintf('\n--- Cumulative feedback (Runs 1--2) to LLM ---\n%s\n', data_fb_cum);

%% 7. Run 3 — Reasoning with cumulative feedback
fprintf('\n========== RUN 3: Reasoning with Cumulative Feedback ==========\n');
fprintf('User: "%s"\n\n', user_input_3);

S3 = call_llm_with_retry(@() llm_reasoning(data_fb_cum, user_input_3, 'ModelName', LLM_MODEL), ...
    MAX_RETRIES, RETRY_DELAY);

phi_llm_returned_3 = llmPhiFromStruct(S3);
[phi_expr_3, phi_llm_validated_3] = validate_phi(S3, DEFAULT_PHI);

fprintf('LLM parameters (updated):\n');
fprintf('  e_max     = %g\n', S3.e_max);
fprintf('  mu_0      = %.2f\n', S3.mu_0);
fprintf('  sigma_0   = %.2f\n', S3.sigma_0);
fprintf('  bar_sigma = %.2f\n', S3.bar_sigma);
fprintf('  init_v    = %g m/s\n', S3.init_v);
fprintf('  phi from LLM (JSON):        %s\n', charOrPlaceholder(phi_llm_returned_3));
fprintf('  phi used in simulation:     %s\n', char(phi_expr_3));

opts3 = struct('mu_0', S3.mu_0, 'sigma_0', S3.sigma_0, ...
    'bar_sigma', S3.bar_sigma, 'init_v', S3.init_v);
fprintf('\nRunning simulation...\n');
results_3 = main_single_run(char(phi_expr_3), S3.e_max, char(SAFETY_METHOD), MU_VALUE, opts3);

%% 8. Trim Run 3 signals (for console summary; full series remain in results_*)
e3     = results_3.state.state(:,11);
speed3 = results_3.state.state(:,1);
prob3  = results_3.prob.prob;

nanidx3 = find(isnan(e3), 1);
if ~isempty(nanidx3)
    e3     = e3(1:nanidx3-1);
    speed3 = speed3(1:nanidx3-1);
    prob3  = prob3(1:nanidx3-1);
end

%% 9. Final console summary (barrier + metrics)
sep_major = repmat('=', 1, 80);
sep_minor = repmat('-', 1, 80);
run_titles = {'Run 1 -- initial inference', 'Run 2 -- after feedback', 'Run 3 -- cumulative feedback'};
users = {user_input_1, user_input_2, user_input_3};
phi_applied = {phi_expr_1, phi_expr_2, phi_expr_3};
phi_llm_raw = {phi_llm_returned_1, phi_llm_returned_2, phi_llm_returned_3};
phi_from_llm = [phi_llm_validated_1, phi_llm_validated_2, phi_llm_validated_3];
emax_runs = [S1.e_max, S2.e_max, S3.e_max];

% Per-run metrics (NaN-trimmed series; E = lateral error magnitude)
max_E = zeros(1, 3);
mean_E = zeros(1, 3);
mean_P = zeros(1, 3);
min_P = zeros(1, 3);
top_speed = zeros(1, 3);
avg_speed = zeros(1, 3);
for r = 1:3
    if r == 1
        ev = e1; pv = prob1; sv = speed1;
    elseif r == 2
        ev = e2; pv = prob2; sv = speed2;
    else
        ev = e3; pv = prob3; sv = speed3;
    end
    max_E(r) = max(abs(ev));
    mean_E(r) = mean(abs(ev));
    mean_P(r) = mean(pv);
    min_P(r) = min(pv);
    top_speed(r) = max(sv);
    avg_speed(r) = mean(sv);
end

fprintf('\n%s\n', sep_major);
fprintf('  FULL RUN REPORT (prompts, LLM phi, e_max, metrics)\n');
fprintf('%s\n', sep_major);
fprintf('  LLM model: %s  |  Safety: %s  |  True friction mu: %.2f\n', ...
    char(LLM_MODEL), char(SAFETY_METHOD), MU_VALUE);
fprintf('  Default phi if fallback: %s\n\n', DEFAULT_PHI);

for r = 1:3
    fprintf('  --- %s ---\n', run_titles{r});
    fprintf('  Prompt (user):\n    "%s"\n\n', char(string(users{r})));
    fprintf('  LLM returned phi_expr: %s\n', charOrPlaceholder(phi_llm_raw{r}));
    fprintf('  Barrier in simulation: %s\n', char(string(phi_applied{r})));
    if phi_from_llm(r)
        src = 'LLM phi accepted (phi_expr_passes_barrier_check)';
    elseif strlength(strtrim(string(phi_llm_raw{r}))) == 0
        src = 'default (no phi_expr in LLM JSON)';
    else
        src = 'default (LLM phi failed checks; see messages above)';
    end
    fprintf('  Source: %s\n', src);
    fprintf('  e_max:  %g m\n', emax_runs(r));
    fprintf('  max |E| [m]: %.4f  |  mean |E| [m]: %.4f\n', max_E(r), mean_E(r));
    fprintf('  min P:        %.4f  |  mean P:       %.4f\n', min_P(r), mean_P(r));
    fprintf('  top speed [m/s]: %.3f  |  avg speed [m/s]: %.3f\n\n', top_speed(r), avg_speed(r));
end

fprintf('%s\n', sep_minor);
fprintf('  Compact table (same metrics)\n');
fprintf('  %-6s %6s %8s %8s %8s %8s %10s %10s %8s\n', ...
    'Run', 'e_max', 'max|E|', 'mean|E|', 'minP', 'meanP', 'topSpd', 'avgSpd', 'LLMphi');
fprintf('  %s\n', repmat('-', 1, 96));
for r = 1:3
    fprintf('  %d      %6g %8.4f %8.4f %8.4f %8.4f %10.3f %10.3f  %s\n', ...
        r, emax_runs(r), max_E(r), mean_E(r), min_P(r), mean_P(r), ...
        top_speed(r), avg_speed(r), shortForConsole(phi_llm_raw{r}, 24));
end
fprintf('%s\n', sep_major);

% Struct for .mat (single place for post-processing / figures)
pipeline_report = struct();
pipeline_report.run_label = run_titles;
pipeline_report.prompt = {char(string(user_input_1)), char(string(user_input_2)), char(string(user_input_3))};
pipeline_report.phi_llm = {char(phi_llm_returned_1), char(phi_llm_returned_2), char(phi_llm_returned_3)};
pipeline_report.phi_applied = {char(phi_expr_1), char(phi_expr_2), char(phi_expr_3)};
pipeline_report.emax = emax_runs;
pipeline_report.max_E = max_E;
pipeline_report.mean_E = mean_E;
pipeline_report.min_P = min_P;
pipeline_report.mean_P = mean_P;
pipeline_report.top_speed_mps = top_speed;
pipeline_report.avg_speed_mps = avg_speed;
pipeline_report.phi_llm_validated = phi_from_llm;

%% 10. Save
% Per-run notes (also inside results_k.state: col1 = Vx speed [m/s], col11 = lateral error e [m];
% results_k.traj = x,y trajectory; results_k.phi_expr / .emax duplicate applied barrier & e_max).
savefile = sprintf('data_mpc/llm_pipeline_%s_%s.mat', ...
    strrep(strrep(char(LLM_MODEL), '-', '_'), '.', '_'), ...
    char(SAFETY_METHOD));
save(savefile, 'results_1', 'results_2', 'results_3', 'S1', 'S2', 'S3', ...
    'user_input_1', 'user_input_2', 'user_input_3', ...
    'data_fb', 'data_fb_2', 'data_fb_cum', ...
    'phi_expr_1', 'phi_expr_2', 'phi_expr_3', ...
    'phi_llm_returned_1', 'phi_llm_returned_2', 'phi_llm_returned_3', ...
    'phi_llm_validated_1', 'phi_llm_validated_2', 'phi_llm_validated_3', ...
    'pipeline_report', 'max_E', 'mean_E', 'min_P', 'mean_P', 'top_speed', 'avg_speed', ...
    'DEFAULT_PHI', ...
    'LLM_MODEL', 'SAFETY_METHOD', 'MU_VALUE');
fprintf('\nSaved: %s\n', savefile);
fprintf('  (results_*, S*, pipeline_report, max_E/mean_E/min_P/mean_P/top_speed/avg_speed, phi_*, feedback, config)\n');

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

function [phi_expr, llm_validated] = validate_phi(S, default_phi)
% llm_validated = true iff the returned phi is the LLM string that passed checks (not default fallback).
% Uses phi_expr_passes_barrier_check (same rule as codes/phi/run_llm_phi_ablation.m; emax = S.e_max).
    llm_validated = false;
    if ~isfield(S, 'phi_expr') || isempty(S.phi_expr)
        fprintf('  LLM phi_expr (rejected): (empty or missing in JSON)\n');
        fprintf('  Using default phi: %s\n', default_phi);
        phi_expr = default_phi;
        return;
    end

    raw = string(S.phi_expr);
    [ok, v0, ve, errMsg] = phi_expr_passes_barrier_check(raw, S.e_max);
    if ok
        phi_expr = raw;
        llm_validated = true;
        fprintf('phi_expr validated: %s  [phi(0)=%.3f, phi(emax)=%.3f]\n', phi_expr, v0, ve);
        return;
    end
    fprintf('  LLM phi_expr (rejected): %s\n', raw);
    if strlength(errMsg) > 0
        fprintf('  Reason: eval error -- %s\n', errMsg);
    else
        fprintf('  Reason: bounds / grid check failed (phi(0)=%.4g, phi(emax)=%.4g)\n', v0, ve);
    end
    fprintf('  Using default phi: %s\n', default_phi);
    phi_expr = default_phi;
end

function s = llmPhiFromStruct(S)
    if isfield(S, 'phi_expr') && ~isempty(S.phi_expr)
        s = string(strtrim(char(string(S.phi_expr))));
    else
        s = "";
    end
end

function s = charOrPlaceholder(t)
    if strlength(strtrim(string(t))) == 0
        s = '(none)';
    else
        s = char(string(t));
    end
end

function s = shortForConsole(str, maxLen)
% Truncate long strings for readable console (no newline wrap).
    c = char(string(str));
    if numel(c) <= maxLen
        s = c;
    else
        s = [c(1:maxLen-3) '...'];
    end
end
