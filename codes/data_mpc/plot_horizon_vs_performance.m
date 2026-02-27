% Plot figures
clear

% Load data
N = 6;
AMPC_files = {'data_AMPC_multi_icy_H05.mat',...
              'data_AMPC_multi_icy_H10.mat',...
              'data_AMPC_multi_icy_H15.mat',...
              'data_AMPC_multi_icy_H20.mat',...
              'data_AMPC_multi_icy_H25.mat',...
              'data_AMPC_multi_icy_H30.mat'};

CDBF_files = {'data_CDBF_multi_icy_H05.mat',...
              'data_CDBF_multi_icy_H10.mat',...
              'data_CDBF_multi_icy_H15.mat',...
              'data_CDBF_multi_icy_H20.mat',...
              'data_CDBF_multi_icy_H25.mat',...
              'data_CDBF_multi_icy_H30.mat'};

APSC_files = {'data_APSC_multi_icy_H05.mat',...
              'data_APSC_multi_icy_H10.mat',...
              'data_APSC_multi_icy_H15.mat',...
              'data_APSC_multi_icy_H20.mat',...
              'data_APSC_multi_icy_H25.mat',...
              'data_APSC_multi_icy_H30.mat'};

AMPC = cell(1, N);
CDBF = cell(1, N);
APSC = cell(1, N);
for i = 1:N
    AMPC{i} = load(AMPC_files{i});
    CDBF{i} = load(CDBF_files{i});
    APSC{i} = load(APSC_files{i});
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Elapsed Time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = figure(2);
clf(f,'reset')
f.Position(3:4) = [600 250];

AMPC_res = zeros(N,1);
CDBF_res = zeros(N,1);
APSC_res = zeros(N,1);
for i=1:N
    AMPC_res(i) = mean( rmmissing(reshape(AMPC{i}.ETIME, [],1)) );  
    CDBF_res(i) = mean( rmmissing(reshape(CDBF{i}.ETIME, [],1)) );
    APSC_res(i) = mean( rmmissing(reshape(APSC{i}.ETIME, [],1)) );   
end
hold on
x = [5, 10, 15, 20, 25, 30];

plot(x, APSC_res, '-o', DisplayName='Proposed')
plot(x, AMPC_res, '-o', DisplayName='AMPC')
plot(x, CDBF_res, '-o', DisplayName='CDBF')
xlabel('Horizon')
ylabel('Time [s]')
xlim([0 30])
%ylim([0,5])
lgd = legend(Location="northwest");
yline(0.2, 'k:', HandleVisibility='off');  
hold off
set(gca, 'FontSize', 14);      % axes tick labels
saveas(f,'fig_horizon_vs_time','epsc')
saveas(f,'fig_horizon_vs_time','png')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Safeprob
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = figure(3);
clf(f,'reset')
f.Position(3:4) = [600 350];

AMPC_res = zeros(N,1);
CDBF_res = zeros(N,1);
APSC_res = zeros(N,1);
for i=1:N
    AMPC_res(i) = mean( rmmissing(reshape(AMPC{i}.PROB, [],1)) );    
    CDBF_res(i) = mean( rmmissing(reshape(CDBF{i}.PROB, [],1)) );    
    APSC_res(i) = mean( rmmissing(reshape(APSC{i}.PROB, [],1)) );    
end
hold on
x = [5, 10, 15, 20, 25, 30];
plot(x, APSC_res, '-o', DisplayName='Proposed')
plot(x, AMPC_res, '-o', DisplayName='AMPC')
plot(x, CDBF_res, '-o', DisplayName='CDBF')
xlabel('Horizon')
ylabel('Safety Probability')
xlim([0 30])
yline(0.9, 'k:', HandleVisibility='off');  
lgd = legend(Location="northwest");
lgd.Position(2) = lgd.Position(2) -0.25;
hold off
set(gca, 'FontSize', 14);      % axes tick labels
saveas(f,'fig_horizon_vs_safeprob','epsc')
saveas(f,'fig_horizon_vs_safeprob','png')
