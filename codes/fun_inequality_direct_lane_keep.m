function cineq = fun_inequality_direct_lane_keep(X,U,e,data,mu,probs)
    emax = get_emax_config();
    n = size(X, 1);
    phi = zeros(n, 1);
    for i = 1:n
        phi(i) = fun_safety_condition(X(i,:)', mu, emax);
    end
    cineq = -phi;
end
