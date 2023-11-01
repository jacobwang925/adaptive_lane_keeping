% Run this file to generate matlab function for  
% lane-keeping nominal controller 

clear

% Folder path for saving generated files
root_dir = matlab.project.rootProject().RootFolder;
save_dir = append(root_dir, '\impl_controller\');


% Linearized lateral model
Cf = 90700;
Cr = 109000;
M  = 1430;
a  = 1.05; % distance from C.M. to front
b  = 1.61; % distance from C.M. to rear 
Iz = 2059.2; 
V0 = 20 * 1000/3600;  % longitudinal velocity

A = [ 0        1            V0              0             0      ; ...
      0 -(Cf+Cr)/(M*V0)      0   (b*Cr-a*Cf)/(M*V0)-V0   Cf/M    ; ...
      0        0             0              1             0      ; ...
      0 (b*Cr-a*Cf)/(Iz*V0)  0  -(a^2*Cf+b^2*Cr)/(Iz*V0) a*Cf/Iz ;
      0        0             0              0             0      ];

B = [0;  0;  0;  0;  1]; 

C = [1, 0, 20, 0, 0]; % correspondes to a lateral preview of approximately 0.7 seconds

sys = ss(A,B,C,0); 

% Solve LQR
R  = 600;
Kp = 5;
Kd = 0.4;
Q  = Kp*transpose(C)*C  + Kd*transpose(C*A)*C*A;
K = lqr(sys,Q,R);

% Controller
syms y nu psi r del rd
x = transpose([ y nu psi r del]);
u = -K*(x - [0; 0; 0; rd; 0]);

matlabFunction(u, Vars={x rd}, File=append(save_dir,'lane_keeping_controller') )

