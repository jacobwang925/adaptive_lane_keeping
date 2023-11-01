function [u, u_nom] = mfun_qp_based_controlelr(p, LfP, LgP, BP, x, paramSF)
% Implementation of MATLAB Function for calculating 
% control input by CBF-based Qadratic Problem 
    
    u_nom = fun_nominal_controller(x);
    
    % objective function
    Q = diag([10,1]);  %eye(length(u_nom));
    H = Q; 
    f = - transpose(u_nom)*Q;
    
    % constraint
    A = -LgP;
    b = LfP + BP + paramSF.alpha * ( p - (1-paramSF.epsilon) );

    % solve QP
    options = optimoptions('quadprog','Algorithm','active-set');
    u = quadprog(H,f,A,b,[],[],[],[],u_nom,options);

end
