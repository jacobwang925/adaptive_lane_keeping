function phi = fun_safety_condition(x, xi)

    phi = lane_keeping(x, xi);
   %phi = tire_forces(x, xi);

end

% Lane Keeping
function phi = lane_keeping(x, xi)

    e    = x(11); % lateral error
    emax = 5; 
    phi = 1 - (e/emax)^2;

end



% IV 22
function phi = tire_forces(x, xi)

    tireF = lugre_tireF(x,xi);
    Ftfl  = sqrt( tireF(1,1)^2 + tireF(1,2)^2);
    Ftfr  = sqrt( tireF(2,1)^2 + tireF(2,2)^2);
    Ftrl  = sqrt( tireF(3,1)^2 + tireF(3,2)^2);
    Ftrr  = sqrt( tireF(4,1)^2 + tireF(4,2)^2);

    eta = 0.4;
    mu  = xi; 
    m = 1430;
    g = 9.8;
    Fsat = mu*m*g/4;

    phi_fl = 1 - (Ftfl/Fsat/eta)^2;
    phi_fr = 1 - (Ftfr/Fsat/eta)^2;
    phi_rl = 1 - (Ftrl/Fsat/eta)^2;
    phi_rr = 1 - (Ftrr/Fsat/eta)^2;
    phi = min( [phi_fl, phi_fr, phi_rl, phi_rr] );

end


