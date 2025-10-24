function [p_param] = gaussian_timeseries(data, pre)
% This function just output pre-calcuated parameters
%
% input: 
%  data: mean and variance of the measurement
%  pre:  mean and variance at the previous step (not used)
%
% output: 
%  p_param: mean and variance at the current step

    % mean and variance of measurement (true values)
    mu_k  = data(1); 
    var_k = data(2);

    % output
    p_param = [mu_k, var_k];

end
