% Run this file to generate matlab function for  
% regulator of longitudinal velocity

clear

% Folder path for saving generated files
root_dir = matlab.project.rootProject().RootFolder;
save_dir = append(root_dir, '\impl_controller\');


% Simplified longitudinal model
Re = 0.325; % Wheel radius [m]
Io = 1.68;  % Wheel moment of inertia [kg*m2]

A = [ 0     Re/(4*Io) ; ...
      0        0      ];

B = [0; 1]; 

C = [1, 0]; 

sys = ss(A,B,C,0); 

% Solve LQR
R  = 1;
Kp = 10^3;
Kd = 1;
Q  = Kp*transpose(C)*C  + Kd*transpose(C*A)*C*A;
K = lqr(sys,Q,R);

% Controller
syms Vx Omega Te Vd
x = transpose([ Vx Te]);
u = -K*(x - [Vd; 0]);

matlabFunction(u, Vars={x Vd}, File=append(save_dir,'longitudinal_regulator') )

