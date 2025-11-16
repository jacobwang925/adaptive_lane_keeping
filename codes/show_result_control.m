% clear all;
% control result

for method = ["APSC", "AMPC", "CDBF"]
% for method = ["CDBF"]   

    for model = ["gpt", "gpt35", "gemini", "gemini20", "deepseek"]
    % for model = ["gpt"]
        % model = "gemini20";
        road = "dry_v";

        filename = method + "_" + model + "_control_" + road + ".mat";
        full_path  = "/Users/dengxiyu/Desktop/adaptive_lane_keeping/LLM/llm_results/"+filename;
        load(full_path);
        thre = 3;

        % GPT
        % load("CDBF_gpt_control_icy.mat");
        % load("CDBF_gpt35_control_icy.mat");

        % Gemini
        % % load("APSC_gemini_control_icy.mat");
        % load("CDBF_gemini20_control_icy.mat")

        % % Deepseek
        % load("CDBF_deepseek_control_icy.mat");


        % fprintf("Prior:");
        % fprintf("Run Aggressive: mean: %.2f, std:%.2f\n", mean(abs(T.MU0_Aggr)), std(abs(T.MU0_Aggr)));
        % fprintf("Run Conservative: mean: %.2f, std:%.2f\n", mean(abs(T.MU0_Cons)), std(abs(T.MU0_Cons)));

        if model == "gpt"
            model_name = "GPT-4o-mini";
        elseif model == "gpt35"
            model_name = "GPT-3.5 Turbo";
        elseif model == "gemini"
            model_name = "Gemini 2.5 Flash";
        elseif model == "gemini20"
            model_name = "Gemini 2.0 Flash";
        elseif model == "deepseek"
            model_name = "DeepSeek-Chat";
        else
            model_name = "Unknown Model";
        end

        fprintf(model_name);
        fprintf("\nLateral:");
        lateral_Aggr = T.Emax_Aggr(T.Emax_Aggr<thre);
        lateral_Cons = T.Emax_Cons(T.Emax_Cons<thre);

        fprintf("Run Clear: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(lateral_Aggr), mean(lateral_Aggr), std(lateral_Aggr));
        fprintf("Run Conservative: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(lateral_Cons), mean(lateral_Cons), std(lateral_Cons));

        fprintf("Speed:");
        speed_Aggr = T.SpeedMean_Aggr(T.Emax_Aggr<thre);
        speed_Cons = T.SpeedMean_Cons(T.Emax_Cons<thre);
        fprintf("Run Clear: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(speed_Aggr), mean(speed_Aggr), std(speed_Aggr));
        fprintf("Run Conservative: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(speed_Cons), mean(speed_Cons), std(speed_Cons));


        diff_lat = round(mean(lateral_Cons),2) - round(mean(lateral_Aggr),2);
        diff_speed = round(mean(speed_Cons),2) - round(mean(speed_Aggr),2);

        fprintf("Difference in Lateral: %.2f\n", diff_lat);
        fprintf("Difference in Speed: %.2f\n", diff_speed);

    end
end