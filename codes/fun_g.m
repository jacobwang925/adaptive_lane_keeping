function [gx] = fun_g(x)

    gx = zeros(length(x), 2);

    x_v = x(1:9); 
    gx(1:9,:) = lugre_gx(x_v, 0); 

end

