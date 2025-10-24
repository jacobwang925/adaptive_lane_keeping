% Run this file to generate matlab functions of 
% the state equation and its jacobians

% vehicle parameters
Cf = 2500*2; % Cornering Stiffness for Front Tire
Cr = 2500*2; % Cornering Stiffness for Rear Tire
b_a = 100; % Aerodynamic Drag Coefficient
b_r = 100; % Tire Drag Coefficient
m = 1200; % Mass of vehicle
g = 9.8;   % Acceleration due to gravity
Fz = m*g; % Weight of vehicle
Lf = 2.5; % Distance from front mid tire axle to vehicle center
Lr = 2.5; % Distance from rear mid tire axle to vehicle center
Iz = 1600.0; % Rotational moment of inertia about the center

% Define symbolic variables
syms Vx Vy r % longitudinal and lateal verocities and yaw rate
syms X Y Psi % vehicle position and angle
syms del    % steering angle 
syms ddel   % time derivative of sterring (control input)
syms Fxf    % longitudinal front tire force
syms dFxf   % time derivative of Fxf (control input)
syms mu     % friction coefficient (unknown parameter)

states = transpose([Vx Vy r  del Fxf X Y Psi]);
inputs = transpose([ddel dFxf]);
parameter = mu;

% tire forces
F_load = b_a * sign(Vx)*Vx^2 + b_r * Vx;
alpha_f = -del +atan( (-Vx*sin(del)+(Vy+Lf*r)*cos(del))/(Vx*cos(del)+(Vy+Lf*r)*sin(del)));
alpha_r = atan( (Vy-Lr*r)/Vx );
Fxr = 0; % assuming rear wheel is not-driving and in quesi-steady state
Fyf = -mu*Fz*tanh( Cf*alpha_f/(mu*Fz) ); % lateral, front
Fyr = -mu*Fz*tanh( Cr*alpha_r/(mu*Fz) ); % lateral, rear
Fsat = mu*m*g/4; 

tireF = sym('tireF', [4, 2]); % [fl, fr, rl, rr], [x, y]
tireF(1,1) = Fxf;  % front left 
tireF(1,2) = Fyf; 
tireF(2,1) = Fxf;  % front right
tireF(2,2) = Fyf; 
tireF(3,1) = Fxr;  % rear left
tireF(3,2) = Fyr; 
tireF(4,1) = Fxr;  % rear right
tireF(4,2) = Fyr;
matlabFunction(tireF,Fsat, File='lateral_tireF', ...
               Vars={states parameter}, ...
               Outputs={'tireF','Fsat'}...
               )

% vehicle dynamics
dxdt = sym('dxdt', [8, 1]);
dxdt(1) =  Vy*r + 1/m*( Fxf*cos(del) -Fyf*sin(del) + Fxr -F_load ) ;
dxdt(2) = -Vx*r + 1/m*( Fyf*cos(del) +Fxf*sin(del) + Fyr );
dxdt(3) = Lf/Iz*( Fyf*cos(del)+Fxf*sin(del) ) -Lr*Fyr;
dxdt(4) = ddel;
dxdt(5) = dFxf;
dxdt(6) = Vx*cos(Psi) - Vy*sin(Psi);
dxdt(7) = Vy*cos(Psi) + Vx*sin(Psi);
dxdt(8) = r;

% generate f(x) and g(x)
fx = subs(dxdt, inputs, zeros(size(inputs)));
gx = jacobian(dxdt, inputs);
matlabFunction(fx, Vars={states parameter}, File='lateral_fx')
matlabFunction(gx, Vars={states parameter}, File='lateral_gx')

%
clear
