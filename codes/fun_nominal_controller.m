function [u] = fun_nominal_controller(x)

   % rate of steering
   Vx    = x(1);
   
   x_r = x(10:12); 
   e_start = 30; % distance of straight road before curve 
   R = 40; % radius of road curvature [m]
   rho = turn90(x_r(1), R, e_start); % road curvature
   rd    = Vx* rho;

   x_lat = [x(11); x(2); x(12); x(3); x(4)];  
   ddel     = lane_keeping_controller(x_lat, rd);

   % longitudinal
%    if Vx <= 40 * 1000/3600 % longitudinal speed [m/s] 
%         dTe = 0.1;  
%    else
%        dTe = 0;
%    end
   Te = x(9);
   dTe =  0.04* (40* 1000/3600 -Vx) - 0.001*Te;

   % control input
   u = [ ddel;  dTe]; 

end
