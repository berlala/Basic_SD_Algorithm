function [x_1,y_1,theta_1] = chassis_update(x_0,y_0,theta_0,v,Psi)
L = 2.5; % wheelbase

x_1 = x_0+v*cos(theta_0); % Theta is the vertical axis angle
y_1 = y_0+v*sin(theta_0);

theta_1 = tan(Psi)/L *v;  %Psi is the steering angle
Yaw_dot = v/l*tan(Psi)
end