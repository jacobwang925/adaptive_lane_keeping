function dwdt = road_coordinate(w, vehicle_dyn, rho)

% road error coordinate
%s = w(1); % distance down the road
%e = w(2); % lateral error
p = w(3); % heading error

% vehicle dynamics
Vx = vehicle_dyn(1);
Vy = vehicle_dyn(2);
r  = vehicle_dyn(3);

% differential equation for error coordinate
dwdt = zeros(3,1);
dwdt(1) = Vx*cos(p) -Vy*sin(p);
dwdt(2) = Vy*cos(p) +Vx*sin(p);
dwdt(3) = r - Vx * rho;
