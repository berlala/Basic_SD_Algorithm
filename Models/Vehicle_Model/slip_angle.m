function [alpha] = slip_angle(x_dot, y_dot, psi_dot, theta, num)

l_f = 1.4;
l_r = 1.5;

if num ==1||num ==2 % front wheel
    alpha = (y_dot+l_f*psi_dot)/x_dot - theta;
else %rear wheel
    alpha = (y_dot-l_r*psi_dot)/x_dot;
end
    
end