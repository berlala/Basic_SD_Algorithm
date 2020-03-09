function [F_y,M_z,t,alpha,alpha_sl]=brush_tyre_lateral(F_z,Vx,Vy)
% condition: cpx = cpy =cp, mu_x = mu_y = mu

% omega: half-drive-axis rotation speed [rad/s]
% Vx: tire frame X speed [m/s]
% Vy: tire frame Y speed [m/s]

rf=0.3;
c_py=6e6; % unit Y stiffness, notice this value has a signifcant affect on the cruve . 15e6 will give good result
mu=1.0;
cz=250000; % unit Z vertical
Ip=2;
eps=0.000001; %% to avoid the deadlock


alpha = -atan(Vy/Vx);

rho=max(0,F_z/cz);
a=0.35*rf*(rho/rf+2.25*sqrt(rho/rf));%% P126 Empirical formula, contact length one side

Theta_y = 2*c_py*a^2/(3*mu*F_z);
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
