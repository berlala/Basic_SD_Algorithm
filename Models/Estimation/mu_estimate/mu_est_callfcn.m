%% Mu Estimate 
% In this file, the Vy, Lateral Force and Yaw-rate dot are directly from the Carsim
% dataset to prove the mu-estimate method is correct.
%% 1. Load Data
close all;
clear;   % 1  for left

%MKZ
%Time,Ax,Ay,Vx,Vy,Yaw,Steer_SW,AVz,X_S1,Y_S1

%data = csvread('40kmh_MKZ0316.csv',2,0); % in linear region
%data = csvread('80kmh_MKZ0316.csv', 2 ,0); %MKZ data
%data = csvread("90kmh_MKZ0316.csv",2,0); % MKZ data with force
%mu_true = 0.85;

%additional test
%data = csvread("35kmh0.2MKZ0317.csv",2,0); mu_true = 0.2;
%data = csvread("50kmh0.2MKZ0316.csv",2,0); mu_true = 0.2;
%data = csvread("45kmh03MKZ0319.csv",2,0); mu_true = 0.3;
%data = csvread("70kmh05MKZ0316.csv",2,0);mu_true = 0.5;
%data = csvread("60kmh06MKZ0319.csv",2,0); mu_true = 0.6;
%data = csvread("85kmh07MKZ0316.csv",2,0);mu_true = 0.7;

%High mu
%data = csvread("70kmh07MKZ0320.csv",2,0); mu_true = 0.7;
%data = csvread("80kmh08MKZ0320.csv",2,0); mu_true = 0.8;
%data = csvread("90kmh07MKZ0320.csv",2,0); mu_true = 0.7;
%data = csvread("90kmh09MKZ0320.csv",2,0); mu_true = 0.9;
%data = csvread("95kmh08MKZ0320.csv",2,0); mu_true = 0.8;
%data = csvread("95kmh10MKZ0320.csv",2,0); mu_true = 1;
%data = csvread("100kmh09MKZ0320.csv",2,0); mu_true = 0.9;
%data = csvread("105kmh10MKZ0320.csv",2,0); mu_true = 1;

% Mu Change Senario
mu_true = 0;
%data = csvread("50kmh0307MKZ0323.csv",2,0);%mu_true =data(:,26);
%data = csvread("50kmh0703MKZ0323.csv",2,0);  %mu_true =data(:,26);
%data = csvread("60kmh04085MKZ0323.csv",2,0); 
data = csvread("60kmh08504MKZ0323.csv",2,0);
%data = csvread("70kmh0307MKZ0323.csv",2,0);
%data = csvread("70kmh0703MKZ0323.csv",2,0);
%data = csvread("90kmh04085MKZ0323.csv",2,0);
%data = csvread("90kmh08504MKZ0323.csv",2,0);


%related parameters
% MKZ Parameters from CarSim, From Yonglong Zhang
sw_ratio = 15.4;
Cf_axle = 2*4.8e4; % Axle, N/rad
Cr_axle = 2*5.5e4; %Axle
lf = 1.328;
lr = 1.482;
ll = lf+lr;
mfl = 5700/9.8;
mfr = mfl;
mf = mfl+mfl;
mrl = 5110/9.8;
mrr = mrl;
mr = mrl+mrr;
m = mr+mf;
Iz = ((lf+lr)*(mf/m))^2*mf + (ll*(mr/m))^2*mr;
%%
%ISO 
Ts = data(1,1); fz = 1/Ts;
vx = data(:,4)/3.6; % [m/s]in VEH, head-lamp direction, default in [km/h]
vy = data(:,5)/3.6;
%psi = data(:,6); 
%ax = data(:,2)*9.8; % [m/s^2] in VEH, head-lamp direction, default in [g]
ay = data(:,3)*9.8; % in VEH, right-hand

psi_dot = data(:,8)/180*3.14; %[rad/s] anticlockwise direction is positive, default in [deg/s]
%yaw_rate_dot = [0;diff(psi_dot)]*fz; % Move to function-inside
steering = data(:,7); % [deg]

Fy_fl =  data(:,14);
Fy_fr =  data(:,15);
Fy_rl =  data(:,16);
Fy_rr =  data(:,17);

Alpha_fl =  data(:,18)/180*pi; %[rad]
Alpha_fr =  data(:,19)/180*pi;
Alpha_rl =  data(:,20)/180*pi;
Alpha_rr =  data(:,21)/180*pi;

disp('MKZ CarSim record All Data have been loaded.')
%% 2. Mu Estimate
 %[mu]  = mu_est(fz, vx, ay, psi_dot, steering)
[mu,vy_obs,Fyf_est,Fyr_est,alpha_f_est,alpha_r_est,Fyf_offline_est,Fyr_offline_est,det_f,det_r]   = ...
    mu_est_debug(fz, vx, ay, psi_dot, steering);

%% 3. Display Debug

plot(Fyf_est,'b');hold on
plot(Fyr_est,'r');
plot(Fyf_offline_est,'b--');
plot(Fyr_offline_est,'r--');

for i  = 1:length(det_f(:,1))
    plot(det_f(i,1), det_f(i,3),'bo')
end
for i  = 1:length(det_r(:,1))
    plot(det_r(i,1), det_r(i,3),'ro')
end
