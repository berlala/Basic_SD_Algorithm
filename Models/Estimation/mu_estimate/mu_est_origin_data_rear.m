%% Mu Estimate 
% In this file, the Vy, Lateral Force and Yaw-rate dot are directly from the Carsim
% dataset to prove the mu-estimate method is correct.
%% 1. Load Data
close all;
clear;   % 1  for left

%MKZ
%Time,Ax,Ay,Vx,Vy,Yaw,Steer_SW,AVz,X_S1,Y_S1

%data = csvread('40kmh_MKZ0316.csv',2,0);
%data = csvread('80kmh_MKZ0316.csv', 2 ,0); %MKZ data
%data = csvread("90kmh_MKZ0316.csv",2,0); % MKZ data with force

%additional test
%data = csvread("35kmh0.2MKZ0317.csv",2,0); mu_true = 0.2;
%data = csvread("50kmh0.2MKZ0316.csv",2,0); mu_true = 0.2;
%data = csvread("45kmh03MKZ0319.csv",2,0); mu_true = 0.3;
%data = csvread("70kmh05MKZ0316.csv",2,0);mu_true = 0.5;
%data = csvread("60kmh06MKZ0319.csv",2,0); mu_true = 0.6;
% data = csvread("85kmh07MKZ0316.csv",2,0);mu_true = 0.7;
%related parameters

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
data = csvread("50kmh0307MKZ0323.csv",2,0);%mu_true =data(:,26);
%data = csvread("50kmh0703MKZ0323.csv",2,0);  %mu_true =data(:,26);
%data = csvread("60kmh04085MKZ0323.csv",2,0); 
%data = csvread("60kmh08504MKZ0323.csv",2,0);
%data = csvread("70kmh0307MKZ0323.csv",2,0);
%data = csvread("70kmh0703MKZ0323.csv",2,0);
%data = csvread("90kmh04085MKZ0323.csv",2,0);
%data = csvread("90kmh08504MKZ0323.csv",2,0);

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
psi = data(:,6); 
ax = data(:,2)*9.8; % [m/s^2] in VEH, head-lamp direction, default in [g]
ay = data(:,3)*9.8; % in VEH, right-hand

psi_dot = data(:,8)/180*3.14; %[rad/s] anticlockwise direction is positive, default in [deg/s]
yaw_rate_dot_cs = data(:,9);
yaw_rate_dot = [0;diff(psi_dot)]*fz;
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
%% 2. Velocity Estimation 
y_obs_log = zeros(length(ax),2);
for i = 1:length(ax)

 theta  = steering(i)/sw_ratio/180*3.14;
 y_mes = [ay(i);psi_dot(i)];
 vx_in = vx(i);
 [y_obs] = vy_direct_fcn(vx_in, y_mes, theta) ;
 
 y_obs_log(i,:) = y_obs';
 
end
vy_obs = y_obs_log(:,1);

%% Force Estimatiuon
for i = 1:length(ay)
 theta  = steering(i)/sw_ratio/180*3.14;
Fyf_est(i) = (m*lr/ll * ay(i) + Iz/ll * yaw_rate_dot(i))/cos(theta);
Fyr_est(i) = m*lf/ll*ay(i)  - Iz/ll*yaw_rate_dot(i);
end

Fyf_cs = Fy_fl +Fy_fr;
Fyr_cs = Fy_rl +Fy_rr;

%%  Slip Angle 
% Calculate The Slip Angle
eps = 0.0000001;

for i = 1:length(vy_obs)
theta  = steering(i)/sw_ratio/180*pi; %[rad]
alpha_f_est(i) = -theta - atan((-vy_obs(i) - lf*psi_dot(i))/(vx(i)+eps)); %[rad]
alpha_r_est(i) = atan((vy_obs(i) - lr*psi_dot(i))/(vx(i)+eps));
end


for i = 1:length(vy)
theta  = steering(i)/sw_ratio/180*pi; %[rad]
alpha_f_cs(i) = -theta - atan((-vy(i) - lf*psi_dot(i))/(vx(i)+eps)); %[rad]
alpha_r_cs(i) = atan((vy(i) - lr*psi_dot(i))/(vx(i)+eps));
end

Alpha_f = (Alpha_fl+Alpha_fr)/2;
Alpha_r = (Alpha_rl+Alpha_rr)/2;

figure(4)
title('Slip Angle Estimation[rad]')
subplot(2,1,1)
plot(Alpha_f); hold on
plot(alpha_f_cs)
plot(alpha_f_est)
legend('Real F','CS F','Est F')
subplot(2,1,2)
plot(Alpha_r); hold on
plot(alpha_r_cs)
plot(alpha_r_est)
legend('Real R','CS R','Est R')

%% Lateral Force Saturation Detection
% The basic idea is compare the Real-Time Overall Lateral Force with the
% Offline Linear Lateral Force Characteristi. If the Real-time value is
% bigger, it should be the lateral force into the non-linear area, which
% means the real-time force value can be used to calculate the mu value.

%%  e.g., The Front Tire from Carsim Data
 deadzone_down  = 1500;
 deadzone_up = 2000;

 Fyr_offline_cs = Cr_axle*-alpha_r_cs;
 data_saturation_cs = [];

for i = 1:length(vy)
    if ((abs(Fyr_offline_cs(i)) - deadzone_down)>abs(Fyr_cs(i))) && (abs(Fyr_cs(i))>400) && ((abs(Fyr_offline_cs(i)) <(abs(Fyr_cs(i))+deadzone_up)))&& (sign(Fyr_offline_cs(i))==sign(Fyr_cs(i)))
       % disp('Detect Saturation!')
        data_saturation_cs = [data_saturation_cs;[i,alpha_r_cs(i), Fyr_offline_cs(i),Fyr_cs(i)]];
    else
    end  
end
disp('Finish Saturation Detection!')

% figure(6)
% plot(Fyr_offline_cs); hold on
% plot(Fyr_cs, 'linewidth',1);
% plot(data_saturation_cs(:,1), data_saturation_cs(:,3),'.')
% title('Lateral Force ')
% legend('Fy Linear CS', 'Fy Real-Time CS','Detected Points CS')
% mu_cs = mean(abs(data_saturation_cs(:,3))/(mf*9.8));

%%   Est Data
 Fyr_offline_est = Cr_axle*-alpha_r_est;
 data_saturation_est = [];

for i = 2:length(vy)
  if ((abs(Fyr_offline_est(i)) - deadzone_down)>abs(Fyr_est(i))) && (abs(Fyr_est(i))>400) && ((abs(Fyr_offline_est(i)) <(abs(Fyr_est(i))+deadzone_up)))&& (sign(Fyr_offline_est(i))==sign(Fyr_est(i)))...
          &&(abs(Fyr_offline_est(i)) > abs(Fyr_offline_est(i-1))) && (abs(alpha_r_est(i))>3.0/180*pi) 
      %  disp('Detect Saturation!')
        data_saturation_est = [data_saturation_est;[i,alpha_r_est(i), Fyr_offline_est(i), Fyr_est(i)]];
    else
    end  
end
disp('Finish Saturation Detection!')
figure(2)
plot(Fyr_offline_est); hold on
plot(Fyr_est);
plot(data_saturation_est(:,1), data_saturation_est(:,3),'o')
legend('Fy Linear EST', 'Fy Real-Time EST','Detected Points EST')
xlabel('Index[-]')
ylabel('Lateral Force [N]')

fprintf('Real Value is %2.2f\n',mu_true)
mu_est_mean_linear = mean(abs(data_saturation_est(:,3))/(mf*9.8))
mu_est_max_linear = max(abs(data_saturation_est(:,3))/(mf*9.8))

%mu_est_mean_realtime = mean(abs(data_saturation_est(:,4))/(mf*9.8))
%mu_est_max_realtime = max(abs(data_saturation_est(:,4))/(mf*9.8))
