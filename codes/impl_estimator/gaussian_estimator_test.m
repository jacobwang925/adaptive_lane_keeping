% Test code for "gaussian_estimator.m"
%
% It also creats a timeseries data 
% used by "gaussian_timeseries.m" (see line 73)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
output = 0;  % output to file (1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Folder path for saving generated files
%root_dir = matlab.project.rootProject().RootFolder;
%save_dir = append(root_dir, '\impl_estimator\');


% time 
t = 0.1:0.1:30;

% mean and variance of measurement (true values)
mu_mes  = 0.7; 
var_mes = 0.1;
sig_mes = sqrt(var_mes);

% mean and variance of prior
mu_0  = 0.3;
var_0 = 0.01;

%%% Initialization %%%%%%%%%%
mu_pre  = mu_0;
var_pre = var_0;
mu_var_list = zeros(length(t)+1, 3);

%%% Parameter Update %%%%%%%%%
% for k = 0: store parameters of prior distribution
mu_var_list(1,:) = [0, mu_pre, var_pre];

% for k = 1: store parameters of prior distribution
mu_var_list(2,:) = [t(1), mu_pre, var_pre];

% for i >= 2: update parameters
for i = 2:length(t)

 % update current mu and variance
 param = gaussian_estimator([mu_mes,var_mes], [mu_pre,var_pre]);
 mu_k  = param(1);
 var_k = param(2); 
 mu_var_list(i+1,:) = [t(i), mu_k, var_k];
 
 % update previsous values to be used in the next step
 mu_pre = mu_k;
 var_pre = var_k;

end


%%% Plot results
% xlim
mu  = 0:1/100:1 ;

% plot prior distribusion
mean = mu_0; var = var_0;
dis = (1/sqrt(2*pi*var))*exp( -(mu-mean).^2 / (2*var) );
plot(mu,dis,'DisplayName','t=0s (prior)')
hold on 
xlabel('\mu')
ylabel('$P_k(\mu | Q_k)$', 'Interpreter','latex')
xlim([mu(1), mu(end)])
%ylim([0,5])

%plot posterior distribusion
k = [11,51,101];
for i = 1:length(k)

    time = mu_var_list(k(i), 1);
    mean = mu_var_list(k(i),2); var = mu_var_list(k(i),3);
    dis = (1/sqrt(2*pi*var))*exp( -(mu-mean).^2 / (2*var) );
    plot(mu,dis,DisplayName= ['t=' num2str(time) 's'] ) 
end
legend('Location','northwest')
hold off

%%% Output timeseries data for reuse by "gaussian_timeseries.m"
mu_var = timeseries(mu_var_list(:,2:3), mu_var_list(:,1));
if output == 1
    save(append(save_dir,'gaussian_timeseries.mat'), "mu_var", '-v7.3')
end