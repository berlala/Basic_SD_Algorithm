%close all;
clear
mass = 580*4;
g = 9.81;
F_z = mass*g/4;

Vx = 30/3.6;
i =1;
for  Vy =-1.5:0.01:1.5

[F_y,M_z,t,alpha, alpha_sl]=brush_tyre_lateral(F_z,Vx,Vy);
 
F_y_log(i) = F_y;
alpha_log(i) = alpha;
i = i+1; 
end

figure(1)
f1 = plot(alpha_log/pi*180, F_y_log,'linewidth',2); hold on
xlabel('Tire Slip Angle  \alpha [Deg]')
ylabel('Lateral Force [N]')
title('2320 Kg Vehicle, Vx = 35km/h')
% 
c_py=15e6;a = 0.0730;
C_Fy = 2*c_py*a^2;
F_y_line =  alpha_log* C_Fy;
f2 = plot(alpha_log/pi*180, F_y_line,'linewidth',2); 
f3 = plot([0.02/pi*180,0.04/pi*180],[2500,5000],'o');
f7 =plot([alpha_sl/pi*180, alpha_sl/pi*180], [-8000, 8000],'r--');
f8 =plot([-alpha_sl/pi*180, -alpha_sl/pi*180], [-8000, 8000],'r--');
axis([-10,10,-8000,8000])
legend([f1,f2, f3],'Brush Tire Model' ,'Current linear Model','Carsim Tire Model')
