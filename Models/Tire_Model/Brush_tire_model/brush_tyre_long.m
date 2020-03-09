function [F_x, kappa] = brush_tyre_long(F_z, Omega,V_x)

%V_x: tire frame X-velocity
%V_sx: tire contact point X- velocity 

mu = 1.0;%friction coff
rf = 0.3; % free rolling
re=0.3; % effective rolling 

cz=250000; % unit Z vertical
rho=max(0,F_z/cz);
a=0.35*rf*(rho/rf+2.25*sqrt(rho/rf));%% P126 Empirical formula, contact length one side
c_px=7e6; % unit X stiffness

kappa = (V_x - Omega*re)/V_x; % longitudinal slip, slip ratio
sigma_x = kappa/(1+kappa); % theoretical slip

C_Fk = 2*c_px*a^2; %P138
Theta_x = 2*c_px*a^2/(3*mu*F_z);

kappa_sl(1) = -1/(1+Theta_x);
kappa_sl(2) = -1/(1-Theta_x);

if kappa < max(kappa_sl) && kappa> min(kappa_sl)
    F_x =  3*mu*F_z*Theta_x*sigma_x*(1-abs(Theta_x*sigma_x)+1/3*(Theta_x*sigma_x)^2);
else
     F_x =mu*F_z*sign(sigma_x);
end

end