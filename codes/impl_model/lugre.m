% Run this file to generate matlab functions of 
% the state equation and its jacobians for
% 3DOF vehicle dynamics with LuGre tire model

% Folder path for saving generated files
root_dir = matlab.project.rootProject().RootFolder;
save_dir = append(root_dir, '\impl_model\');

% vehicle parameters
m  = 1430; % Mass of vehicle [kg]
Iz = 2059.2; % Yaw moment of inertia [kg*m2]
g = 9.8;  % Acceleration due to gravity
Fz = m*g/4; % Weight of vehicle
Lf = 1.05;  % Distance from front mid tire axle to vehicle center [m]
Lr = 1.61;  % Distance from rear mid tire axle to vehicle center [m]
d  = 1.55;  % Vehicle width [m] 
Re = 0.325; % Wheel radius [m]
Io = 1.68;  % Wheel moment of inertia [kg*m2]
sigma0x = 195;   % Longitudinal rubber stiffness [1/m]
sigma2x = 0.001; % Longitudinal relative viscous [s/m]
kx      = 13.4; % Longitudinal load ditribution factor 
sigma0y = 195;   % Lateral rubber stiffness [1/m]
sigma2y = 0.001; % Lateral relative viscous [s/m]
ky      = 13.4; % Lateral  load ditribution factor 
mu_s = 0.55; % static friction coefficient
mu_c = 0.35; % dynamic friction coefficient
Vs   = 6.6;  % Stribeck relative velocity [m/x]
C1   = 1;    % Coefficient of simplified Stribeck function
C2   = 0.52; 
C3   = 0.1;  

% Define symbolic variables
syms Vx Vy r  real % longitudinal and lateal verocities and yaw rate
syms del      real % steering angle 
syms Te       real % Torque to the wheel
syms Ofl Ofr Orl Orr  real % tire angular velocities
syms ddel     real  % time derivative of sterring (control input)
syms dTe       real % time derivative of torque (control input)
syms mu       real  % friction coefficient (unknown parameter)

states = transpose([Vx Vy r del Ofl Ofr Orl Orr Te]);
inputs = transpose([ddel dTe]);
parameter = mu;

% slip angle and slip ratio
alpha_f = del -(Vy+Lf*r)/Vx ;
alpha_r = - (Vy-Lr*r)/Vx ;
lambda_fl = (Re*Ofl - Vx)/max([Re*Ofl,Vx]);
lambda_fr = (Re*Ofr - Vx)/max([Re*Ofr,Vx]);
lambda_rl = (Re*Orl - Vx)/max([Re*Orl,Vx]);
lambda_rr = (Re*Orr - Vx)/max([Re*Orr,Vx]);

slipA = sym('slipA', [2, 1]); % [fl, fr, rl, rr]
slipA(1) = alpha_f;  
slipA(2) = alpha_r; 
matlabFunction(slipA, File=append(save_dir,'lugre_slipA'), ...
               Vars={states}, Outputs={'slipA'})

slipR = sym('slipR', [4, 1]); % [fl, fr, rl, rr]
slipR(1) = lambda_fl;  
slipR(2) = lambda_fr; 
slipR(3) = lambda_rl; 
slipR(4) = lambda_rr; 
matlabFunction(slipR, File=append(save_dir,'lugre_slipR'), ...
               Vars={states}, Outputs={'slipR'}, Optimize=false)

% LuGre tire model
g_st = @(vr) mu_c + (mu_s - mu_c)*exp(-(vr/Vs)^0.5);
Fx = @(lambda,alpha,vr,omega) ( sigma0x/(sigma0x*vr/(mu*g_st(vr)) +kx*Re*abs(omega)) +sigma2x ) ...
                               *max(Re*omega,Vx)*lambda*Fz;
Fy = @(lambda,alpha,vr,omega) ( sigma0y/(sigma0y*vr/(mu*g_st(vr)) +ky*Re*abs(omega)) +sigma2y ) ...
                               *Vx*alpha*Fz;

% % LuGre tire model (simplified)
% g_st = @(lambda, alpha) C1 - C2*lambda - C3*alpha;
% Fx = @(lambda,alpha,vr,omega) ( sigma0x/(sigma0x*vr/(mu*g_st(lambda,alpha)) +kx*Re*omega) +sigma2x ) ...
%                                *max(Re*omega,Vx)*lambda*Fz;
% Fy = @(lambda,alpha,vr,omega) ( sigma0y/(sigma0y*vr/(mu*g_st(lambda,alpha)) +ky*Re*omega) +sigma2y ) ...
%                                *Vx*alpha*Fz;

vr_fl = norm( [max(Re*Ofl,Vx)*lambda_fl, Vx*alpha_f] );
vr_fr = norm( [max(Re*Ofr,Vx)*lambda_fr, Vx*alpha_f] );
vr_rl = norm( [max(Re*Orl,Vx)*lambda_rl, Vx*alpha_r] );
vr_rr = norm( [max(Re*Orr,Vx)*lambda_rr, Vx*alpha_r] );

Fxfl = Fx(lambda_fl, alpha_f, vr_fl, Ofl);
Fxfr = Fx(lambda_fr, alpha_f, vr_fr, Ofr);
Fxrl = Fx(lambda_rl, alpha_r, vr_rl, Orl);
Fxrr = Fx(lambda_rr, alpha_r, vr_rr, Orr);
Fyfl = Fy(lambda_fl, alpha_f, vr_fl, Ofl);
Fyfr = Fy(lambda_fr, alpha_f, vr_fr, Ofr);
Fyrl = Fy(lambda_rl, alpha_r, vr_rl, Orl);
Fyrr = Fy(lambda_rr, alpha_r, vr_rr, Orr);

tireF = sym('tireF', [4, 2]); % [fl, fr, rl, rr], [x, y]
tireF(1,1) = Fxfl;  % front left 
tireF(1,2) = Fyfl; 
tireF(2,1) = Fxfr;  % front right
tireF(2,2) = Fyfr; 
tireF(3,1) = Fxrl;  % rear left
tireF(3,2) = Fyrl; 
tireF(4,1) = Fxrr;  % rear right
tireF(4,2) = Fyrr;
matlabFunction(tireF, File=append(save_dir,'lugre_tireF'), ...
               Vars={states parameter}, ...
               Outputs={'tireF'}...
               )

F_load = 0; 

% vehicle dynamics
dxdt = sym('dxdt', [length(states), 1]);
dxdt(1) =  Vy*r + 1/m*( (Fxfl+Fxfr)*cos(del) -(Fyfl+Fyfr)*sin(del) + Fxrl+Fxrr -F_load );
dxdt(2) = -Vx*r + 1/m*( (Fyfl+Fyfr)*cos(del) +(Fxfl+Fxfr)*sin(del) + Fyrl+Fyrr );
dxdt(3) = Lf/Iz*( (Fyfl+Fyfr)*cos(del)+(Fxfl+Fxfr)*sin(del) ) -Lr/Iz*(Fyrl+Fyrr) ...
          +d/2* ( (Fxfr-Fxfl)*cos(del)+(Fyfl-Fyfr)*sin(del) + (Fxrr-Fxrl) ) ;
dxdt(4) = ddel;
dxdt(5) = 1/Io * (-Re*Fxfl + Te/4 -0*Ofl);
dxdt(6) = 1/Io * (-Re*Fxfr + Te/4 -0*Ofr);
dxdt(7) = 1/Io * (-Re*Fxrl + Te/4 -0*Orl);
dxdt(8) = 1/Io * (-Re*Fxrr + Te/4 -0*Orr);
dxdt(9) = 1000*dTe;

% generate f(x) and g(x)
fx = subs(dxdt, inputs, zeros(size(inputs)));
gx = jacobian(dxdt, inputs);
matlabFunction(fx, Vars={states parameter}, File=append(save_dir,'lugre_fx') )
matlabFunction(gx, Vars={states parameter}, File=append(save_dir,'lugre_gx') )
