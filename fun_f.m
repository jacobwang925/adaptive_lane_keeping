function [fx] = fun_f(x, xi)
% vector field f the state equation given by 
% dxdt = f(x,xi) + g(u,xi) u

    fx = zeros(size(x));

    % vehicle dynamics
    x_v = x(1:9); 
    fx(1:9)  = lugre_fx(x_v,xi);

    % road
    x_r = x(10:12); 
    e_start = 30; % distance of straight road before curve 
    R = 40; % radius of road curvature [m]
    rho = turn90(x_r(1), R, e_start); % road curvature
    fx(10:12) = road_coordinate(x_r,x_v,rho); 

end
