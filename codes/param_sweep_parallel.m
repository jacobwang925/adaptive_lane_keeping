% Script to run closed-loop simulation with chosen parameters

clear; clc;

% --- Define Parameter Lists for Sweeping ---
% prior_ic1_list = [0.3, 0.5, 0.9];
% prior_ic2_list = [0.05, 0.3];
% mes_var_list   = [0.05, 0.3];
% emax_list      = [3, 5, 10];

prior_ic1_list = [0.3];
prior_ic2_list = [0.05];
mes_var_list   = [0.05];
emax_list      = [5, 10];

% --- Set Save Flag ---
saveData = true;

disp('=== Starting parameter sweep ===')

% --- Nested Loops for All Parameter Combinations ---
run_index = 1;
for p1_val = prior_ic1_list
    for p2_val = prior_ic2_list
        for mv_val = mes_var_list
            for em_val = emax_list
                
                % Define parameters for this specific run
                prior_ic = [p1_val, p2_val];
                mes_var  = mv_val;
                emax     = em_val;
                
                % Display progress to the command window
                fprintf('Running sim #%d: prior_ic=[%.2f, %.2f], mes_var=%.2f, emax=%.1f\n', ...
                        run_index, prior_ic(1), prior_ic(2), mes_var, emax);
                
                % Run the parallel simulation
                % The function will handle saving data internally
                run_closed_loop_parallel(prior_ic, mes_var, emax, saveData);
                
                run_index = run_index + 1;
                
            end
        end
    end
end

disp('=== All simulations complete ===')