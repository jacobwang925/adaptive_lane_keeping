% Plot figures
clear

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AMPC_icy = load("data_AMPC_multi_icy_H05.mat");
% CDBF_icy = load("data_CDBF_multi_icy_H05.mat");
% APSC_icy = load("data_APSC_multi_icy_H05.mat");
% AMPC_icy = load("data_mpc/data_AMPC_multi_icy_H10_prior_0p3_0p01_mesvar_0p1_emax_15.mat");
% CDBF_icy = load("data_mpc/data_CDBF_multi_icy_H10_prior_0p3_0p01_mesvar_0p1_emax_15.mat");
% APSC_icy = load("data_mpc/data_APSC_multi_icy_H10_prior_0p3_0p01_mesvar_0p1_emax_15.mat");
AMPC = load("data_mpc/data_AMPC_multi_icy_H10_prior_0p3_0p05_mesvar_0p05_emax_10.mat");
CDBF = load("data_mpc/data_CDBF_multi_icy_H10_prior_0p3_0p05_mesvar_0p05_emax_10.mat");
APSC = load("data_mpc/data_APSC_multi_icy_H10_prior_0p3_0p05_mesvar_0p05_emax_10.mat");


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot trajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f = figure(1);
clf(f,'reset')
f.Position(3:4) = [400 400];
pbaspect([1 1 1])
hold on

num = 10;

% For legends
i = 1;

% Adaptive PSC (Proposed)
traj_x = rmmissing( APSC.TRAJ(i, 1:end-1, 1) );
traj_y = rmmissing( APSC.TRAJ(i, 1:end-1, 2) );
plot(traj_x, traj_y, Color='#0072BD', DisplayName='Proposed')

% Adaptive MPC
traj_x = rmmissing( AMPC.TRAJ(i, :, 1) );
traj_y = rmmissing( AMPC.TRAJ(i, :, 2) );
plot(traj_x, traj_y, Color='#D95319', DisplayName='Adaptive MPC')

% Adaptive MPC + CDBF
traj_x = rmmissing( CDBF.TRAJ(i, 1:end-1, 1) );
traj_y = rmmissing( CDBF.TRAJ(i, 1:end-1, 2) );
plot(traj_x, traj_y, Color='#EDB120', DisplayName='CDBF')


for i = 1:num

    % Adaptive MPC
    traj_x = rmmissing( AMPC.TRAJ(i, :, 1) );
    traj_y = rmmissing( AMPC.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#D95319', HandleVisibility='off')

    % Adaptive MPC + CDBF
    traj_x = rmmissing( CDBF.TRAJ(i, 1:end-1, 1) );
    traj_y = rmmissing( CDBF.TRAJ(i, 1:end-1, 2) );
    plot(traj_x, traj_y, Color='#EDB120', HandleVisibility='off')

    % Adaptive PSC (Proposed)
    traj_x = rmmissing( APSC.TRAJ(i, 1:end-1, 1) );
    traj_y = rmmissing( APSC.TRAJ(i, 1:end-1, 2) );
    plot(traj_x, traj_y, Color='#0072BD', HandleVisibility='off')
end

t = 0:0.1:1;
left_x = [t*30  30+35*cos(pi/2*(t-1)) t*0+65]; 
left_y = [t*0+5 40+35*sin(pi/2*(t-1)) t*30+40]; 
left  = plot(left_x, left_y, Color='k', HandleVisibility='off');
right_x = [t*30  30+45*cos(pi/2*(t-1)) t*0+75 ]; 
right_y = [t*0-5 40+45*sin(pi/2*(t-1)) t*30+40 ]; 
right = plot(right_x, right_y, Color='k', HandleVisibility='off');

hold off
xlim([0, 90])
ylim([-20, 70])
ylim([-20, 150])
xlabel('Distance (m)')
ylabel('Distance (m)')
legend(Location="northwest")
saveas(f,'fig_trajectory','epsc')
saveas(f,'fig_trajectory','png')
