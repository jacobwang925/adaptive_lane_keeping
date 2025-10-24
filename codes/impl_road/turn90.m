function rho = turn90(s,R, start)
%TURN 

if s < start
    rho = 0; 
elseif s <= start + (pi/2)*R  
    rho = 1/R;
else
    rho = 0;

end

