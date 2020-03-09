%Fiala Model
function [F_y,alpha]=fiala_tyre_lateral(F_z,x_dot, y_dot, psi_dot, theta , i_wheel)

mu = 1.0;
Ca = 1.5e5;%[ N/rad]
lf = 1.3472;
lr = 1.5028;

if i_wheel ==1 || i_wheel == 2
alpha = (y_dot+lf*psi_dot)/x_dot - theta;
%Theta_y = 2*Ca*a^2/(3*mu*F_z);
elseif i_wheel == 3 || i_wheel ==4
    alpha = (y_dot - lr*psi_dot)/x_dot;
   % Theta_y = 2*Ca*a^2/(3*mu*F_z);
end

if abs(alpha) <3*mu*F_z/Ca
    F_y = -Ca*tan(alpha) + Ca^2/(3*mu*F_z)*abs(tan(alpha))*tan(alpha) - Ca^3/(27*mu^2*F_z^2)*(tan(alpha))^3;
else
    F_y = -mu*F_z*sign(alpha);
end

end