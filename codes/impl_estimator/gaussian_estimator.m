function [p_param] = gaussian_estimator(data, pre)
% Implementation of Bayesian parameter estimator 
% with Gaussian distributions
%
% input: 
%  data: mean and variance of the measurement
%  pre:  mean and variance at the previous step
%
% output: mean and variance at the current step

    % mean and variance of measurement (true values)
    mu_mes  = data(1); 
    var_mes = data(2);
    sig_mes = sqrt(var_mes);

    % mean and variance at the previous step
    mu_pre =  pre(1);
    var_pre = pre(2);

    % update current mu and variance
    X_k = mu_mes + sig_mes*randn;
    mu_k  = (var_mes * mu_pre + var_pre * X_k) / (var_pre + var_mes);
    var_k = (var_mes * var_pre )/( var_pre + var_mes);

    % output
    p_param = [mu_k, var_k];

end
