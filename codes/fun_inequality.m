function cineq = fun_inequality(X,U,e,data,mu,probs)
    u = U(1,:)';
    epsilon = 0.1;  alpha   = 1.0; 
    p = probs(1); LfP = probs(2); LgP = probs(3:4); BP = probs(5);
    cineq = -LgP * u - LfP - BP - alpha * ( p - (1-epsilon) );
    %disp([cineq, e])
    %cineq = -1;
end
