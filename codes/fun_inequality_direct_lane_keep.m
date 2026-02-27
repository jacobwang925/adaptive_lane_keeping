function cineq = fun_inequality_direct_lane_keep(X,U,e,data,mu,probs)
    
    e= X(:,11);
    emax = 5; 
    phi = 1 - (e/emax).^2;
    cineq = -phi;
    disp(cineq')
    %cineq = -1;
end
