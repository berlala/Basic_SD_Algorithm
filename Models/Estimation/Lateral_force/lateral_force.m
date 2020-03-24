% Lateral Force Estimation Method
%% Method 1 (without the effect of the longititual force)
%% Load Data
close all;
clear;   % 1  for left

%MKZ
%Time,Ax,Ay,Vx,Vy,Yaw,Steer_SW,AVz,X_S1,Y_S1
%data = csvread('40kmh_dlc_MKZ0309.csv', 2 ,0); %MKZ data
%data = csvread('80kmh_dlc_MKZ0309.csv', 2 ,0); %MKZ data
%data = csvread('90kmh_dlc_MKZ0309.csv', 2 ,0); %MKZ data
%data = csvread("20kmh_MKZ0310_1.csv",2,0); % MKZ data update with Force
data = csvread("40kmh_MKZ0310_F.csv",2,0); % MKZ data update with Force
%data = csvread("80kmh_MKZ0310_F.csv",2,0);

%related parameters

% Parameters from CarSim
sw_ratio = 15.4;
lf = 1.328;
lr = 1.482;
ll = lf+lr;
mf = 580*2;
mr = 580*2;
m = mr+mf;

Iz = ((lf+lr)*(mf/m))^2*mf + (ll*(mr/m))^2*mr;
%%
%ISO 
Ts = data(1,1); fz = 1/Ts
vx = data(:,4)/3.6; % [m/s]in VEH, head-lamp direction, default in [km/h]
vy = data(:,5)/3.6;
psi = data(:,6); 
ax = data(:,2)*9.8; % [m/s^2] in VEH, right hand direction, default in [g]
ay = data(:,3)*9.8; % in VEH, right-hand

psi_dot = data(:,8)/180*3.14; %[rad/s] anticlockwise direction is positive, default in [deg/s]
yaw_rate_dot = [0;diff(psi_dot)]*fz;
steering = data(:,7); % [deg]

% If there is Force Data
yaw_rate_dot_real = data(:,9);
figure(1)
plot(yaw_rate_dot);hold on
plot(yaw_rate_dot_real,'-','linewidth',0.8)
ylabel('Yaw Rate Dot[rad/s^2]')
title('Compare Diff Method and Real')
legend('Diff Yaw Rate Dot','Carsim Yaw Rate Dot')

Fy_fl =  data(:,14);
Fy_fr =  data(:,15);
Fy_rl =  data(:,16);
Fy_rr =  data(:,17);

disp('MKZ CarSim record All Data have been loaded.')
%%

ll = lf+lr;
Fyf = [];
Fyr = [];
for i = 1:length(ay)
 theta  = steering(i)/sw_ratio/180*3.14;
Fyf(i) = (m*lr/ll * ay(i) + Iz/ll * yaw_rate_dot(i))/cos(theta);
Fyr(i) = m*lf/ll*ay(i)  - Iz/ll*yaw_rate_dot(i);
end

%Carsim Real
Fyf_real = Fy_fl + Fy_fr;
Fyr_real = Fy_rl + Fy_rr;

figure(2)
plot(Fyf);hold on
plot(Fyr)
plot(Fyf_real,'-','linewidth',1.2);
plot(Fyr_real,'-','linewidth',1.2);

xlabel('Index[-]')
ylabel('Lateral Force [N] ')
legend('Obs Front Axle','Obs Rear Axle', 'Carsim F-Axle', 'Carsim R-Axle')
title('Estimate Axle Force @ 20km/h')
