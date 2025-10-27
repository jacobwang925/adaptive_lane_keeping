% llm code


% Script to run closed-loop simulation with chosen parameters
%
clear; clc;

% Define parameters
prior_ic = [0.80 0.01];   % initial prior for estimator
mes_var  = 0.80;          % measurement variance
e_max    = 5.0;           % maximum lateral error threshold

disp('=== Running closed-loop simulation ===')
res = run_closed_loop_single(prior_ic, mes_var, e_max);

disp('=== Simulation complete ===')

% Example: inspect results
disp('Final risk probability:')
disp(res.prob(end,:))

% You can optionally save results
% save('results_case1.mat','res')
