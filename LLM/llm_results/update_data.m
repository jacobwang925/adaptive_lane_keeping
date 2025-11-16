% update data file with new data
currentPath = pwd;
parentPath = fileparts(currentPath);
for method = ["APSC", "AMPC", "CDBF"]
    for model = ["gpt", "gpt35", "gemini", "gemini20", "deepseek"]
        road = "dry_v";

        filename = method + "_" + model + "_control_" + road + ".mat";
        load(filename);

        for i = 1:100
            idx        = T{i,11};
            filename   = T{i,12};
            full_path  = parentPath + "/codes/"+filename;
    
            data = load(full_path);
            
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

            % T{i,11} = idx;
            % T{i,12} = filename;
            T{i,13} = e_max;
            T{i,14} = e_mean;
            T{i,15} = speed_max;
            T{i,16} = speed_mean;
            T{i,17} = prob_min;
            T{i,18} = prob_mean;
        end


    end
end