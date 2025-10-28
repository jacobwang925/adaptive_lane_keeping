% llm code


% Script to run closed-loop simulation with chosen parameters
%
clear; clc;

% Define parameters
prior_ic = [0.30 0.01];   % initial prior for estimator
mes_var  = 0.10;          % measurement variance
emax    = 15.0;           % maximum lateral error threshold

disp('=== Running closed-loop simulation ===')
res = run_closed_loop_single(prior_ic, mes_var, emax);

disp('=== Simulation complete ===')

% Example: inspect results
disp('Final risk probability:')
disp(res.prob(end,:))

% You can optionally save results
% save('results_case1.mat','res')
