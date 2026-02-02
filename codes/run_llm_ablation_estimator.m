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
    road = "icy";
    horizon = 10;

    % LLM model
    for llm = ["gemini25", "gemini20", "deepseek"]
        % gpt
        if llm == "gpt4o"
            model = "gpt-4o-mini";
            matname = parentPath + "/LLM/llm_results/" + method+"_gpt_estimator_unsure_"+road+"_v.mat";
        end

        if llm == "gpt35"
            model = "gpt-3.5-turbo";
            matname = parentPath + "/LLM/llm_results/" + method+"_gpt35_estimator_unsure_"+road+"_v.mat";
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
            matname = parentPath + "/LLM/llm_results/" + method+"_deepseek_estimator_unsure_"+road+"_v.mat";
        end


        % --- Define Parameter Lists for Sweeping ---
        lists.prior_ic1_list = [0.3, 0.5, 0.9];
        lists.prior_ic2_list = [0.05, 0.3];
        lists.mes_var_list   = [0.05, 0.3];
        lists.emax_list      = [3, 5, 10];


        % llm code
        disp('=== LLM mapping user instruction to initial guess ===')
        % user_input = ["i need to slow down and be conservative. i want to keep it very safe. The road is dry",
        %     "runid want aggressive performance and keep a high speed.",
        %     ""];

        % Script to run closed-loop simulation with chosen parameters

        disp('=== Running closed-loop simulation ===')


        T = cell2table(cell(0,22), 'VariableNames', { ...
            'Dry_Input','Scenario_Dry','File_Dry', 'MU0_Dry', 'MUe_Dry', 'Emax_Dry','Emean_Dry','SpeedMax_Dry','SpeedMean_Dry','ProbMin_Dry','ProbMean_Dry', ...
            'No_Input','Scenario_No', 'File_No', 'MU0_No', 'MUe_No', 'Emax_No','Emean_No','SpeedMax_No','SpeedMean_No','ProbMin_No','ProbMean_No' ...
            });


        load(parentPath + "/LLM/user_inputs/input_dry_unsure.mat");

        for i = 1:100

            row = {[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]};
            fprintf('=== Episode: %d ===\n', i)
            input1 = input_dry_unsure(i);
            row{1,1} = input1;
            input2 = input_dry_unsure(i);
            row{1,12} = input2;

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
                            fprintf('Attempt %d failed: %s\n', attempt, ME.message);
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

                filename = sprintf("data_mpc/data_%s_multi_%s_H%d_prior_%s_%s_mesvar_%s_emax_%d.mat", ...
                    method, road, horizon, num2str(mu0), num2str(sig0), num2str(mes_var), emax);

                APSC = load(filename);


                fprintf('=== Reasoning for the run: %d ===', runid)
                idx = randi([1, 20]);

                fprintf("\nrun: %d, scenario: %d, loadfile: %s\n", runid, idx, filename);
                % pngname = char("run" + runid);
                % f = plot_single_traj(APSC, idx, pngname);

                e = APSC.STATE(idx,1:400,11);

                e_valid = e(~isnan(e));
                e_abs = abs(e_valid);
                e_peak = quantile(e_abs, 0.995);
                e_max = max(e_abs);
                e_mean = mean(e_abs);

                mu = APSC.MU(idx);

                e_str = "[" + join(string(round(e_valid,4)), ",") + "]";
                speed = APSC.SPEED(idx, 1:400);

                speed_valid = speed(~isnan(speed));
                speed_max = max(speed_valid);
                speed_mean = mean(speed_valid);

                speed_str = "[" + join(string(round(speed_valid,4)), ",") + "]";
                prob = APSC.PROB(idx, 1:400);
                prob_valid = prob(~isnan(prob));
                prob_min =  min(prob_valid);
                prob_mean = mean(prob_valid);

                prob_str = "[" + join(string(round(prob_valid,4)), ",") + "]";


                if runid == 1
                    row{1,2} = idx;
                    row{1,3} = filename;
                    row{1,4} = mu0;
                    row{1,5} = mu;
                    row{1,6} = e_max;
                    row{1,7} = e_mean;
                    row{1,8} = speed_max;
                    row{1,9} = speed_mean;
                    row{1,10} = prob_min;
                    row{1,11} = prob_mean;
                elseif runid == 2
                    row{1,13} = idx;
                    row{1,14} = filename;
                    row{1,15} = mu0;
                    row{1,16} = mu;
                    row{1,17} = e_max;
                    row{1,18} = e_mean;
                    row{1,19} = speed_max;
                    row{1,20} = speed_mean;
                    row{1,21} = prob_min;
                    row{1,22} = prob_mean;
                else
                    disp("error")
                end

                % data_fb = sprintf([ ...
                %     "Run %d summary:\n" + ...
                %     "- Lane error 99.5%% quantile (e_peak)=%.3f, previous e_max is assigned as %.2f.\n" + ...
                %     "- All the lateral error data: %s\n" + ...
                %     "- Average safety probability=%.2f (minimum=%.2f).\n" + ...
                %     "- All the safety probability data: %s.\n" + ...
                %     "- Maximum speed=%.2f m/s, estimated road friction mu=%.2f.\n" + ...
                %     "- All the speed data: %s\n" + ...
                %     "- Duration: %d steps.\n"], ...
                %     runid, e_peak, prev.e_max, e_str, prob_mean, prob_min, prob_str, speed_max, mu, speed_str, length(e_valid));

                data_fb = sprintf([ ...
                    "Run %d summary:\n" + ...
                    "- Lane error 99.5%% quantile (e_peak)=%.3f, previous e_max is assigned as %.2f.\n" + ...
                    "- Average safety probability=%.2f (minimum=%.2f).\n" + ...
                    "- Maximum speed=%.2f m/s, estimated road friction mu=%.2f.\n" + ...
                    "- Duration: %d steps.\n"], ...
                    runid, e_peak, prev.e_max, prob_mean, prob_min, speed_max, mu, length(e_valid));
            end

            T = [T; row];

            % parallel run
            % saveData = true;
            % res = run_closed_loop_parallel(prior_ic, mes_var, emax, saveData);
        end

        disp('=== Simulation complete ===')
        save(matname, 'T');
        fprintf('Save to: '+ matname);

        % inspect results
        % disp('Final risk probability:')
        % disp(res.prob(end,:))

    end
end