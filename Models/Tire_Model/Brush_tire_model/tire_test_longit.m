close all;clear
mass = 580*4;
g = 9.81;
F_z = mass*g/4;

Vx = 35/3.6;
i =1;
for  Omega =0:0.01:Vx/0.3
[F_x,kappa]=brush_tyre_long(F_z,Omega,Vx);
 
Omega_log(i) = Omega;
F_x_log(i) = F_x;
kappa_log(i) = kappa;
i = i+1; 
end

C_Fk =   7.4682e+04;

figure(1)
plot(kappa_log, F_x_log,'linewidth',2); hold on;
plot(kappa_log, C_Fk*kappa_log,'linewidth',2);
grid on;
xlabel('Slip Ratio  \kappa [-]')
ylabel('Longitudinal Force [N]')
axis([0,1, 0,6000])
legend('Brush Tire Model', 'Linear Model')
title('2320 Kg Vehicle')

figure(2) % kappa< 0.2 is stable region
plot( kappa_log, Omega_log*0.3,'linewidth',2); 
grid on 
ylabel('Relative Ground Speed [m/s]')
xlabel('Slip Ratio  \kappa [-]')

