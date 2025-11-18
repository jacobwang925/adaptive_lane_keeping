% Estimator result

clear all;
currentPath = pwd;
parentPath = fileparts(currentPath);
road = "unsure_icy";

for method = ["APSC", "AMPC", "CDBF"]
    for model = ["gpt4o", "gpt35", "gemini25", "gemini20", "deepseek"]

        filename = method + "_" + model + "_estimator_" + road + ".mat";
        full_path  = parentPath+"/LLM/llm_results/"+filename;
        load(full_path);
        thre = 3;

        if model == "gpt4o"
            model_name = "GPT-4o-mini";
        elseif model == "gpt35"
            model_name = "GPT-3.5 Turbo";
        elseif model == "gemini25"
            model_name = "Gemini 2.5 Flash";
        elseif model == "gemini20"
            model_name = "Gemini 2.0 Flash";
        elseif model == "deepseek"
            model_name = "DeepSeek-Chat";
        else
            model_name = "Unknown Model";
        end


        fprintf(model_name);
        fprintf("\nPrior:\n");
        fprintf("Run Clear: mean: %.2f, std:%.2f\n", mean(abs(T.MU0_Dry)), std(abs(T.MU0_Dry)));
        fprintf("Run Clear estimator: mean: %.2f, std:%.2f\n", mean(abs(T.MUe_Dry)), std(abs(T.MUe_Dry)));

        fprintf("Run Vague: mean: %.2f, std:%.2f\n", mean(abs(T.MU0_No)), std(abs(T.MU0_No)));
        fprintf("Run Vague estimator: mean: %.2f, std:%.2f\n", mean(abs(T.MUe_No)), std(abs(T.MUe_No)));

        fprintf("Lateral:\n");
        lateral_dry = T.Emax_Dry(T.Emax_Dry<thre);
        lateral_no = T.Emax_No(T.Emax_No<thre);

        fprintf("Run Clear: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(lateral_dry), mean(lateral_dry), std(lateral_dry));
        fprintf("Run Vague: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(lateral_no), mean(lateral_no), std(lateral_no));

        fprintf("Speed:\n");
        speed_dry = T.SpeedMax_Dry(T.Emax_Dry<thre);
        speed_no = T.SpeedMax_No(T.Emax_No<thre);
        fprintf("Run Clear: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(speed_dry), mean(speed_dry), std(speed_dry));
        fprintf("Run Vague: feasible runs: %d,: mean: %.2f, std:%.2f\n", length(speed_no), mean(speed_no), std(speed_no));
    end
end