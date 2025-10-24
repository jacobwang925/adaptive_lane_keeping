function cineq = fun_inequality_CDBF(X,U,e,data,mu,probs)
    u = U(1,:)';
    Vx = X(2,1);
    Vy = X(2,2);
    r  = X(2,3);
    del= X(2,4);
    %Te = X(5);
    Lf = 1.05;  % Distance from front mid tire axle to vehicle center [m]
    Lr = 1.61;  % Distance from rear mid tire axle to vehicle center [m]
    b1 = -1; c1 =  1.0;
    b2 = -1; c2 = -1.0;
    b3 =  1; c3 =  1.0;
    b4 =  1; c4 = -1.0;
    s1 = Vx*Lr/(Lf+Lr).*del;
    s2 = Vx/(Lf+Lr).*del;
    h1 = b1.*(Vy-s1) +c1 -(r-s2); 
    h2 = (r-s2)  -(b2.*(Vy-s1) +c2); 
    h3 = b3.*(Vy-s1) +c3 -(r-s2); 
    h4 = (r-s2)  -(b4.*(Vy-s1) +c4); 
    cineq = [-h1; -h2];
    %disp(cineq')
    %cineq = -1;
end
