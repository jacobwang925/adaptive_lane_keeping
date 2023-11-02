% Plot figures
clear

% load data
safeAdap  = load("data_SafeAdap_mu02.mat");
fixedSafe = load("data_FixedSafe_mu02.mat");
nominal   = load("data_Nominal_mu02.mat");

% initialize figure index
fnum = 1;

% safe probability
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [600 160];
hold on
plot(seconds(nominal.prob.Time),  nominal.prob.prob,Color='r', DisplayName='Nominal controller')
plot(seconds(fixedSafe.prob.Time),  fixedSafe.prob.prob,Color='m', DisplayName='Non-adaptive controller')
plot(seconds(safeAdap.prob.Time), safeAdap.prob.prob,Color='b', DisplayName='Safe adaptive controller')
pmin = yline(0.9, Color='k', LineStyle=':');
pmin.HandleVisibility = 'off';
hold off
xlabel('Time / s')
ylabel('Safe probability')
xlim([0, 30])
legend(Location="southeast")
saveas(f,'fig_mu02_prob','epsc')

% longitudinal velocity
fnum=fnum+1;
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [600 160];
hold on
plot(seconds(nominal.state.Time),   nominal.state.state(:,1)*3600/1000,   Color='r', DisplayName='Nominal controller')
plot(seconds(fixedSafe.state.Time), fixedSafe.state.state(:,1)*3600/1000, Color='m', DisplayName='Non-adaptive controller')
plot(seconds(safeAdap.state.Time),  safeAdap.state.state(:,1)*3600/1000,  Color='b', DisplayName='Safe adaptive controller')

hold off
xlabel('Time / s')
ylabel('Longitud. velocity / km/h')
xlim([0, 30])
ylim([0 40])
%legend(Location="best")
saveas(f,'fig_mu02_velocity','epsc')

% Steering angle
fnum=fnum+1;
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [600 160];
hold on
plot(seconds(nominal.state.Time),   nominal.state.state(:,4)*180/pi,   Color='r',  DisplayName='Nominal controller')
plot(seconds(fixedSafe.state.Time), fixedSafe.state.state(:,4)*180/pi, Color='m',  DisplayName='Non-adaptive controller')
plot(seconds(safeAdap.state.Time),  safeAdap.state.state(:,4)*180/pi,  Color='b',  DisplayName='Safe adaptive controller')
hold off
xlabel('Time / s')
ylabel('Steering angle / deg')
xlim([0, 30])
ylim([-30, 150])
%legend(Location="northwest")
saveas(f,'fig_mu02_steer_angle','epsc')

% Lateral error
fnum=fnum+1;
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [600 160];
hold on
plot(seconds(nominal.state.Time),   nominal.state.state(:,11),   Color='r', DisplayName='Nominal controller')
plot(seconds(fixedSafe.state.Time), fixedSafe.state.state(:,11), Color='m', DisplayName='Non-adaptive controller')
plot(seconds(safeAdap.state.Time),  safeAdap.state.state(:,11),  Color='b', DisplayName='Safe adaptive controller')

lmax = yline(-5, Color='k', LineStyle=':');
lmax.HandleVisibility = 'off';
hold off
xlabel('Time / s')
ylabel('Lateral error / m')
xlim([0, 30])
ylim([-15 5])
%legend(Location="best")
saveas(f,'fig_mu02_lat_error','epsc')


% % Parameter estimation
% f4 = figure(4);
% clf(f4,'reset')
% f4.Position(3:4) = [600 160];
% hold on
% plot(seconds(safe.dist.Time), safe.dist.dist(:,1),  Color='b', DisplayName='Mean')
% ylabel('Estimated mean of \xi')
% %
% yyaxis right
% plot(seconds(safe.dist.Time), safe.dist.dist(:,2), Color='r', DisplayName='Variance')
% ylim([0,0.0001])
% ylabel('Variance of \xi')
% hold off
% xlabel('Time / s')
% legend(Location="northwest")
% saveas(f4,'fig_est.eps','png')

% % input
% f5 = figure(5);
% clf(f5,'reset')
% f5.Position(3:4) = [600 160];
% hold on
% plot(seconds(nom.input.Time),  nom.input.input(:,1),  Color='m', DisplayName='\mu = 0.40')
% plot(seconds(safe.input.Time), safe.input.input(:,1), Color='b', DisplayName='\mu = 0.35')
% hold off
% ylim([0,0.06])
% xlabel('Time / s')
% ylabel('Input / rad s^{-1}')
% legend(Location="northeast")
% saveas(f5,'fig_input.eps','png')