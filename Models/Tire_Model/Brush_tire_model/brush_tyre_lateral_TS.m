function [F_y,M_z,t,alpha,alpha_sl]=brush_tyre_lateral_TS(F_z,x_dot, y_dot, psi_dot, theta , i_wheel)
% condition: cpx = cpy =cp, mu_x = mu_y = mu

% omega: half-drive-axis rotation speed [rad/s]
% Vx: tire frame X speed [m/s], x_dot is the long velocity from IMU
% Vy: tire frame Y speed [m/s], y_dot is the lateral velocity from IMU

rf=0.3;
% front and rear
c_py_f=  14e6; % UNIT Y stiffness, default 6e6, Notice the unit. for single tire
c_py_r = 15e6;
mu=1.0; % road firction [-] 
cz=250000; % unit Z vertical, default 250000
Ip=2;
eps=0.000001; %% to avoid the deadlock
lf = 1.3472;
lr = 1.5028;

x_dot = x_dot +eps;
rho=max(0,F_z/cz);
a=0.35*rf*(rho/rf+2.25*sqrt(rho/rf));%% P126 Empirical formula, contact length one side

% calculate for slip angle from chassis velocity;
%1 and 2 for FL and FR, 3 and 4 for RL and RR;
% [2*c_py_f*a^2] is the equivalent stiffness
if i_wheel ==1 || i_wheel == 2
    alpha = (y_dot+lf*psi_dot)/x_dot - theta;
    Theta_y = 2*c_py_f*a^2/(3*mu*F_z); %P131, only a combination for parameter here, front tyre
elseif i_wheel == 3 || i_wheel ==4
    alpha = (y_dot - lr*psi_dot)/x_dot;
    Theta_y = 2*c_py_r*a^2/(3*mu*F_z); %P131, only a combination for parameter here
end


sigma_y = tan(alpha);

alpha_sl = atan(1/Theta_y);

if abs(alpha)<alpha_sl
    F_y = 3*mu*F_z*Theta_y*sigma_y*(1-abs(Theta_y*sigma_y)+1/3*(Theta_y*sigma_y)^2);
    M_z = -mu*F_z*a*Theta_y*sigma_y*(1-3*abs(Theta_y*sigma_y)+3*(Theta_y*sigma_y)^2-abs(Theta_y*sigma_y)^3);
else
    F_y =mu*F_z*sign(alpha);
    M_z = 0;
end

t = -M_z/F_y;
return
