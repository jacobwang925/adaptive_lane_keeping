function dxdt = fun_system_dynamics(x,u,mu,probs)

 dxdt = fun_f(x,mu) + fun_g(x)* u;

end

