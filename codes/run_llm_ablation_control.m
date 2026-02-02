clear; clc; close all;

% Please set you api keys here
setenv('OPENAI_API_KEY','');   
setenv('X-goog-api-key','');
setenv('deepseekApiKey','');

% --- Helper functions ---
function S_new = map_to_closest(S, lists)
% Helper for nearest value
nearest = @(x, list) list(find(abs(list - x) == min(abs(list - x)), 1));

% Initialize
S_new = S;

% Map each expected field if it exists
if isfield(S, 'mu_0')
    S_new.mu_0 = nearest(S.mu_0, lists.prior_ic1_list);
end
if isfield(S, 'sigma_0')
    S_new.sigma_0 = nearest(S.sigma_0, lists.prior_ic2_list);
end
if isfield(S, 'bar_sigma')
    S_new.bar_sigma = nearest(S.bar_sigma, lists.mes_var_list);
end
if isfield(S, 'e_max')
    S_new.e_max = nearest(S.e_max, lists.emax_list);
end
end

function road = map_mu_to_road(mu0)
if mu0 > 0.7
    road = "dry";
elseif mu0 > 0.4
    road = "normal";
else
    road = "icy";
end
end

function s = num2str(x)
% Convert numeric value to string, preserving all digits (no rounding)
str = char(string(x));          % preserves numeric precision printed by MATLAB
s = strrep(str, '.', 'p');      % replace '.' with 'p'
end

maxRetries = 5;
retryDelay = 2;
currentPath = pwd;
parentPath = fileparts(currentPath);

for method = ["APSC", "AMPC", "CDBF"]
    road = "dry";
    horizon = 10;

    % LLM model

    % --- Define Parameter Lists for Sweeping ---
    lists.prior_ic1_list = [0.3, 0.5, 0.9];
    lists.prior_ic2_list = [0.05, 0.3];
    lists.mes_var_list   = [0.05, 0.3];
    lists.emax_list      = [3, 5, 10];


    for llm = ["gpt4o", "gpt35", "gemini25", "gemini20", "deepseek"]
        % gpt
        if llm == "gpt4o"
            model = "gpt-4o-mini";
            matname = parentPath + "/LLM/llm_results/" + method+"_gpt_control_"+road+"_v.mat";
        end

        if llm == "gpt35"
            model = "gpt-3.5-turbo";
            matname = parentPath + "/LLM/llm_results/" + method+"_gpt35_control_"+road+"_v.mat";
        end

        % gemini
        if llm == "gemini25"
            model = "gemini-2.5-flash";
            matname = parentPath + "/LLM/llm_results/" + method+"_gemini_estimator_unsure_"+road+"_v.mat";
        end

        if llm == "gemini20"
            model = "gemini-2.0-flash";
            matname = parentPath + "/LLM/llm_results/" + method+"_gemini20_estimator_unsure_"+road+"_v.mat";
        end

        % deepseek
        if llm == "deepseek"
            model = "deepseek-chat";
            url = "https://api.deepseek.com/chat/completions" + model;
            matname = parentPath + "/LLM/llm_results/" + method+"_deepseek_control_"+road+"_v.mat";
        end

        % llm code
        disp('=== LLM mapping user instruction to initial guess ===')

        % Script to run closed-loop simulation with chosen parameters

        disp('=== Running closed-loop simulation ===')

        T = cell2table(cell(0,18), 'VariableNames', { ...
            'Aggressive_Input','Scenario_Aggr','File_Aggr', 'Emax_Aggr','Emean_Aggr','SpeedMax_Aggr','SpeedMean_Aggr','ProbMin_Aggr','ProbMean_Aggr', ...
            'Conservative_Input','Scenario_Cons', 'File_Cons', 'Emax_Cons','Emean_Cons','SpeedMax_Cons','SpeedMean_Cons','ProbMin_Cons','ProbMean_Cons' ...
            });

        load(parentPath + "/LLM/user_inputs/input_aggressive.mat");
        load(parentPath + "/LLM/user_inputs/input_conservative.mat");

        for i = 1:100

            row = {[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]};
            fprintf('=== Episode: %d ===\n', i)
            input1 = input_aggressive(i);
            row{1,1} = input1;
            input2 = input_conservative(i);
            row{1,10} = input2;

            for runid = 1:2

                fprintf('Run: %d\n', runid)
                if runid == 1
                    for attempt = 1:maxRetries
                        try
                            S = llm_inferring(input1, 'ModelName', model);
                            % If it reaches here, it succeeded — exit the loop
                            fprintf('✅ LLM call succeeded on attempt %d.\n', attempt);
                            break;
                        catch ME
                            % If this is not the last attempt, retry after delay
                            fprintf('⚠️ Attempt %d failed: %s\n', attempt, ME.message);
                            if attempt < maxRetries
                                fprintf('Retrying in %.1f seconds...\n', retryDelay);
                                pause(retryDelay);
                            else
                                % If all retries failed, rethrow the last error
                                fprintf('❌ All %d attempts failed.\n', maxRetries);
                                rethrow(ME);
                            end
                        end
                    end
                    user_input = input1;
                    % pause(1.0);
                else
                    user_fb = "Hows the driving?" + input2;
                    user_input = input2;
                    for attempt = 1:maxRetries
                        try
                            S = llm_reasoning(data_fb, input2, 'ModelName', model);
                            % If it reaches here, it succeeded — exit the loop
                            fprintf('✅ LLM call succeeded on attempt %d.\n', attempt);
                            break;
                        catch ME
                            % If this is not the last attempt, retry after delay
                            fprintf('⚠️ Attempt %d failed: %s\n', attempt, ME.message);
                            if attempt < maxRetries
                                fprintf('Retrying in %.1f seconds...\n', retryDelay);
                                pause(retryDelay);
                            else
                                % If all retries failed, rethrow the last error
                                fprintf('❌ All %d attempts failed.\n', maxRetries);
                                rethrow(ME);
                            end
                        end
                    end
                    % pause(1.0);
                end

                % Define parameters
                prev = map_to_closest(S, lists);
                mu0 = prev.mu_0;
                sig0 = prev.sigma_0;

                prior_ic = [prev.mu_0 prev.sigma_0];
                mes_var = prev.bar_sigma;
                emax = prev.e_max;


                feedback = sprintf( ...
                    "User Input: %s\n" + ...
                    "LLM guess: e_max=%.2f, mu_0=%.2f, sigma_0=%.2f, bar_sigma=%.2f\n" + ...
                    "LLM Mapped: e_max=%.2f, mu_0=%.2f, sigma_0=%.2f, bar_sigma=%.2f", ...
                    user_input, S.e_max, S.mu_0, S.sigma_0, S.bar_sigma, emax, mu0, sig0, mes_var);
                fprintf(feedback);

                fprintf("\nThe groundtruth road condition is: %s, the guess road condition is %s.\n", road, map_mu_to_road(mu0));


                if S.init_v == 10
                    fprintf("choose init v = 10🚨\n");
                    filename = sprintf("data_mpc/data_%s_multi_%s_H%d_prior_%s_%s_mesvar_%s_emax_%d_v0_10.mat", ...
                        method, road, horizon, num2str(mu0), num2str(sig0), num2str(mes_var), emax);
                else
                    filename = sprintf("data_mpc/data_%s_multi_%s_H%d_prior_%s_%s_mesvar_%s_emax_%d.mat", ...
                        method, road, horizon, num2str(mu0), num2str(sig0), num2str(mes_var), emax);
                end

                data = load(filename);


                fprintf('=== Reasoning for the run: %d ===', runid)
                idx = randi([1, 20]);

                fprintf("\nrun: %d, scenario: %d, loadfile: %s\n", runid, idx, filename);

                e = data.STATE(idx,1:400,11);
                speed = data.SPEED(idx, 1:400);
                prob = data.PROB(idx, 1:400);

                nanidx = find(isnan(e), 1);   % find index of first NaN
                if ~isempty(nanidx)
                    e_valid = e(1:nanidx-1);
                    speed_valid = speed(1:nanidx-1);
                    prob_valid = prob(1:nanidx-1);
                else
                    e_valid = e;
                    speed_valid = speed;
                    prob_valid = prob;
                end
                e_abs = abs(e_valid);
                e_peak = quantile(e_abs, 0.995);
                e_max = max(e_abs);
                e_mean = mean(e_abs);

                mu = data.MU(idx);

                % e_str = "[" + join(string(round(e_valid,4)), ",") + "]";

                speed_max = max(speed_valid);
                speed_mean = mean(speed_valid);

                % speed_str = "[" + join(string(round(speed_valid,4)), ",") + "]";
                prob_min =  min(prob_valid);
                prob_mean = mean(prob_valid);

                % prob_str = "[" + join(string(round(prob_valid,4)), ",") + "]";

                if runid == 1
                    row{1,2} = idx;
                    row{1,3} = filename;
                    row{1,4} = e_max;
                    row{1,5} = e_mean;
                    row{1,6} = speed_max;
                    row{1,7} = speed_mean;
                    row{1,8} = prob_min;
                    row{1,9} = prob_mean;
                elseif runid == 2
                    row{1,11} = idx;
                    row{1,12} = filename;
                    row{1,13} = e_max;
                    row{1,14} = e_mean;
                    row{1,15} = speed_max;
                    row{1,16} = speed_mean;
                    row{1,17} = prob_min;
                    row{1,18} = prob_mean;
                else
                    disp("error")
                end

                e_str    = char(strjoin(compose("%.3f", e_valid(:).'), ", "));
                prob_str = char(strjoin(compose("%.2f", prob_valid(:).'), ", "));
                speed_str= char(strjoin(compose("%.2f", speed_valid(:).'), ", "));

                data_fb = sprintf([ ...
                    'Run %d summary:\n' ...
                    '- Lane error 99.5%% quantile (e_peak)=%.3f, previous e_max=%.2f.\n' ...
                    '- All the lateral error data: %s\n' ...
                    '- Average safety probability=%.2f (minimum=%.2f).\n' ...
                    '- All the safety probability data: %s\n' ...
                    '- Maximum speed=%.2f m/s, estimated road friction mu=%.2f.\n' ...
                    '- All the speed data: %s\n' ...
                    '- Duration: %d steps.\n' ], ...
                    runid, e_peak, prev.e_max, e_str, prob_mean, prob_min, prob_str, ...
                    speed_max, mu, speed_str, numel(e_valid));
            end

            % data_fb = sprintf([ ...
            %     "Run %d summary:\n" + ...
            %     "- Lane error 99.5%% quantile (e_peak)=%.3f, previous e_max is assigned as %.2f.\n" + ...
            %     "- Average safety probability=%.2f (minimum=%.2f).\n" + ...
            %     "- Maximum speed=%.2f m/s, estimated road friction mu=%.2f.\n" + ...
            %     "- Duration: %d steps.\n"], ...
            %     runid, e_peak, prev.e_max, prob_mean, prob_min, speed_max, mu, length(e_valid));
            % end

            T = [T; row];

            % parallel run
            % saveData = true;
            % res = run_closed_loop_parallel(prior_ic, mes_var, emax, saveData);
        end

        disp('=== Simulation complete ===')

        save(matname, 'T');
        fprintf('Save to: '+ matname);
    end
end

% inspect results
% disp('Final risk probability:')
% disp(res.prob(end,:))

