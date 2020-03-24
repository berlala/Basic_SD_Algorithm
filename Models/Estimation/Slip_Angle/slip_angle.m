%% Slip Angle 
%% Load Data
close all;
clear;   % 1  for left
% Tire Real front : 5e4, Rear: 

% CarSim MKZ
%data = csvread('40kmh_MKZ0316.csv',2,0);
data = csvread('80kmh_MKZ0316.csv',2,0);
%
sw_ratio = 15.4;
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
%pose data
vx = data(:,4)/3.6; % in VEH, head-lamp dlrection
vy = data(:,5)/3.6; 

psi = data(:,6); 
ax = data(:,2)*9.8; % [m/s^2] in VEH, right hand dlrection, default in [g]
ay = data(:,3)*9.8; % in VEH, head-lamp
psi_dot = data(:,8)/180*3.14; %[rad/s] anticlockwise dlrection is positive, default in [deg/s]

steering = data(:,7); % [deg]% [percentage]
theta  = steering/sw_ratio/180*3.14; %[rad]
Ts = data(1,1); fz = 1/Ts;
psi_ddot = [0;diff(psi_dot)]*fz;


Alpha_fl =  data(:,18)/180*pi; %[rad]
Alpha_fr =  data(:,19)/180*pi;
Alpha_rl =  data(:,20)/180*pi;
Alpha_rr =  data(:,21)/180*pi;

Alpha_f = (Alpha_fl+Alpha_fr)/2;
Alpha_r = (Alpha_rl+Alpha_rr)/2;

disp(' All Data have been loaded.')

%%
% Valid for Carsim data

for i = 1:length(vy)
theta  = steering(i)/sw_ratio/180*pi; %[rad]
alpha_f_est(i) = -theta - atan((-vy(i) - lf*psi_dot(i))/(vx(i)+eps)); %[rad]
alpha_r_est(i) = atan((vy(i) - lr*psi_dot(i))/(vx(i)+eps));
end

%%
figure(1)
subplot(2,1,1)
plot(Alpha_f); hold on
plot(alpha_f_est)
legend('Real F','Est F')
subplot(2,1,2)
plot(Alpha_r); hold on
plot(alpha_r_est)
legend('Real R','Est R')

