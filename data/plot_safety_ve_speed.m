% Plot figures
clear

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load data
safeAdap_dry = load("data_SafeAdap_multi_dry.mat");
fixdSafe_dry = load("data_FixedSafe_multi_dry.mat");
nomCtrol_dry = load("data_Nominal_multi_dry.mat");

safeAdap_wet = load("data_SafeAdap_multi_wet.mat");
fixdSafe_wet = load("data_FixedSafe_multi_wet.mat");
nomCtrol_wet = load("data_Nominal_multi_wet.mat");

safeAdap_icy = load("data_SafeAdap_multi_icy.mat");
fixdSafe_icy = load("data_FixedSafe_multi_icy.mat");
nomCtrol_icy = load("data_Nominal_multi_icy.mat");


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot safety
P_safeAdap_dry = mean( rmmissing(reshape(safeAdap_dry.PROB, [],1)) );
P_fixdSafe_dry = mean( rmmissing(reshape(fixdSafe_dry.PROB, [],1)) );
P_nomCtrol_dry = mean( rmmissing(reshape(nomCtrol_dry.PROB, [],1)) );

P_safeAdap_wet = mean( rmmissing(reshape(safeAdap_wet.PROB, [],1)) );
P_fixdSafe_wet = mean( rmmissing(reshape(fixdSafe_wet.PROB, [],1)) );
P_nomCtrol_wet = mean( rmmissing(reshape(nomCtrol_wet.PROB, [],1)) );

P_safeAdap_icy = mean( rmmissing(reshape(safeAdap_icy.PROB, [],1)) );
P_fixdSafe_icy = mean( rmmissing(reshape(fixdSafe_icy.PROB, [],1)) );
P_nomCtrol_icy = mean( rmmissing(reshape(nomCtrol_icy.PROB, [],1)) );


fnum = 1;
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [430 450];
hold on
X = categorical({'Dry','Wet','Icy'});
X = reordercats(X,{'Dry','Wet','Icy'});
data = [P_safeAdap_dry, P_fixdSafe_dry, P_nomCtrol_dry;
        P_safeAdap_wet, P_fixdSafe_wet, P_nomCtrol_wet;
        P_safeAdap_icy, P_fixdSafe_icy, P_nomCtrol_icy];
bar(X, data)
pmin = yline(0.9, Color='k', LineStyle=':');
pmin.HandleVisibility = 'off';
hold off
xlabel('Road condition')
ylabel('Safe probability')
saveas(f,'fig_safe_vs_roadcond','epsc')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot speed
V_safeAdap_dry = mean( rmmissing(reshape(safeAdap_dry.SPEED, [],1)) );
V_fixdSafe_dry = mean( rmmissing(reshape(fixdSafe_dry.SPEED, [],1)) );
V_nomCtrol_dry = mean( rmmissing(reshape(nomCtrol_dry.SPEED, [],1)) );

V_safeAdap_wet = mean( rmmissing(reshape(safeAdap_wet.SPEED, [],1)) );
V_fixdSafe_wet = mean( rmmissing(reshape(fixdSafe_wet.SPEED, [],1)) );
V_nomCtrol_wet = mean( rmmissing(reshape(nomCtrol_wet.SPEED, [],1)) );

V_safeAdap_icy = mean( rmmissing(reshape(safeAdap_icy.SPEED, [],1)) );
V_fixdSafe_icy = mean( rmmissing(reshape(fixdSafe_icy.SPEED, [],1)) );
V_nomCtrol_icy = mean( rmmissing(reshape(nomCtrol_icy.SPEED, [],1)) );


fnum = fnum+1;
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [430 450];
hold on
X = categorical({'Dry','Wet','Icy'});
X = reordercats(X,{'Dry','Wet','Icy'});
data = [V_safeAdap_dry, V_fixdSafe_dry, V_nomCtrol_dry;
        V_safeAdap_wet, V_fixdSafe_wet, V_nomCtrol_wet;
        V_safeAdap_icy, V_fixdSafe_icy, V_nomCtrol_icy]*3600./1000 ;
bar(X, data)
pmin.HandleVisibility = 'off';
hold off
xlabel('Road condition')
ylabel('Longitudinal speed (km/h)')
ylim([0,  35])
legend({'Proposed safe controller', 'Non-adaptive safe controller', 'Nominal controller'} , Location='northeast')
saveas(f,'fig_speed_vs_roadcond','epsc')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot safe vs speed
safeAdap_PROB  = [safeAdap_dry.PROB;  safeAdap_icy.PROB];
safeAdap_SPEED = [safeAdap_dry.SPEED; safeAdap_icy.SPEED]*3600./1000;
fixdSafe_PROB  = [fixdSafe_dry.PROB;  fixdSafe_icy.PROB];
fixdSafe_SPEED = [fixdSafe_dry.SPEED; fixdSafe_icy.SPEED]*3600./1000;
nomCtrol_PROB  = [nomCtrol_dry.PROB;  nomCtrol_wet.PROB;  nomCtrol_icy.PROB];
nomCtrol_SPEED = [nomCtrol_dry.SPEED; nomCtrol_wet.SPEED; nomCtrol_icy.SPEED]*3600./1000;

fnum = fnum+1;
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [400 350];
hold on

num = height(safeAdap_PROB);
for i = 1:num
    P = mean( rmmissing(safeAdap_PROB(i,:) ));
    V = mean( rmmissing(safeAdap_SPEED(i,:) ));
    scatter(P,V, "MarkerFaceColor", "#0072BD", "MarkerEdgeColor","k");
end

num = height(fixdSafe_PROB);
for i = 1:num
    P = mean( rmmissing(fixdSafe_PROB(i,:) ));
    V = mean( rmmissing(fixdSafe_SPEED(i,:) ));
    scatter(P,V, "MarkerFaceColor", "#D95319", "MarkerEdgeColor","k")
end

num = height(nomCtrol_PROB);
for i = 1:num
    P = mean( rmmissing(nomCtrol_PROB(i,:) ));
    V = mean( rmmissing(nomCtrol_SPEED(i,:) ));
    scatter(P,V, "MarkerFaceColor", "#EDB120", "MarkerEdgeColor","k")
end

hold off
xlabel('Safety probability')
ylabel('Longitudinal speed (km/h)')
saveas(f,'fig_safe_vs_speed','epsc')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot trajectories

fnum=fnum+1;
f = figure(fnum);
clf(f,'reset')
f.Position(3:4) = [400 400];
pbaspect([1 1 1])
hold on

num = 10;

% for legends
traj_x = rmmissing( safeAdap_icy.TRAJ(1, :, 1) );
traj_y = rmmissing( safeAdap_icy.TRAJ(1, :, 2) );
plot(traj_x, traj_y, Color='#0072BD', DisplayName='Proposed safe controller')

traj_x = rmmissing( fixdSafe_icy.TRAJ(1, :, 1) );
traj_y = rmmissing( fixdSafe_icy.TRAJ(1, :, 2) );
plot(traj_x, traj_y, Color='#D95319', DisplayName='Non-adaptive controller')

traj_x = rmmissing( nomCtrol_icy.TRAJ(1, :, 1) );
traj_y = rmmissing( nomCtrol_icy.TRAJ(1, :, 2) );
plot(traj_x, traj_y, Color='#EDB120', DisplayName='Nominal controller')

for i = 1:num

    % nominal
    traj_x = rmmissing( nomCtrol_icy.TRAJ(i, :, 1) );
    traj_y = rmmissing( nomCtrol_icy.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#EDB120', HandleVisibility='off')

    traj_x = rmmissing( nomCtrol_dry.TRAJ(i, :, 1) );
    traj_y = rmmissing( nomCtrol_dry.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#EDB120', HandleVisibility='off')

    traj_x = rmmissing( nomCtrol_wet.TRAJ(i, :, 1) );
    traj_y = rmmissing( nomCtrol_wet.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#EDB120', HandleVisibility='off')

    % fixed
    traj_x = rmmissing( fixdSafe_icy.TRAJ(i, :, 1) );
    traj_y = rmmissing( fixdSafe_icy.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#D95319', HandleVisibility='off')

    traj_x = rmmissing( fixdSafe_wet.TRAJ(i, :, 1) );
    traj_y = rmmissing( fixdSafe_wet.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#D95319', HandleVisibility='off')

    traj_x = rmmissing( fixdSafe_dry.TRAJ(i, :, 1) );
    traj_y = rmmissing( fixdSafe_dry.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#D95319', HandleVisibility='off')

    % adaptive
    traj_x = rmmissing( safeAdap_icy.TRAJ(i, :, 1) );
    traj_y = rmmissing( safeAdap_icy.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#0072BD', HandleVisibility='off')

    traj_x = rmmissing( safeAdap_wet.TRAJ(i, :, 1) );
    traj_y = rmmissing( safeAdap_wet.TRAJ(i, :, 2) );
    plot(traj_x, traj_y, Color='#0072BD', HandleVisibility='off')

    traj_x = rmmissing( safeAdap_dry.TRAJ(i, :, 1) );
    traj_y = rmmissing( safeAdap_dry.TRAJ(i, :, 2) );
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
xlabel('Distance (m)')
ylabel('Distance (m)')
legend(Location="northwest")
saveas(f,'fig_trajectory','epsc')


