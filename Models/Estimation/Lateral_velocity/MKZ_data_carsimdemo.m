%% Load Data
close all;
clear;   % 1  for left

%MKZ
%Time,Ax,Ay,Vx,Vy,Yaw,Steer_SW,AVz,X_S1,Y_S1
data = csvread('40kmh_dlc_MKZ0309.csv', 2 ,0); %MKZ data
%data = csvread('80kmh_dlc_MKZ0309.csv', 2 ,0); %MKZ data
%data = csvread('90kmh_dlc_MKZ0309.csv', 2 ,0); %MKZ data
%related parameters

% Parameters from CarSim
sw_max = 29 ; %[deg], max front wheel steering angle
sw_ratio = 15.4;
Cf = 1e5; %Axle 
Cr = 1.1e5;
lf = 1.328;
lr = 1.482;
%%
%ISO 
vx = data(:,4)/3.6; % [m/s]in VEH, head-lamp direction, default in [km/h]
vy = data(:,5)/3.6;
psi = data(:,6); 
ax = data(:,2)*9.8; % [m/s^2] in VEH, right hand direction, default in [g]
ay = data(:,3)*9.8; % in VEH, right-hand

psi_dot = data(:,8)/180*3.14; %[rad/s] anticlockwise direction is positive, default in [deg/s]
steering = data(:,7); % [deg]
disp('MKZ CarSim record All Data have been loaded.')

% %% Change the data from MAP Frame to VEH Frame (x_dot/y_dot are in vehicle frame)
% for index = 1:length(T_pose_ind)
%     i = T_pose_ind(index); 
%     x_dot(index)  =vx(i)*sin(psi(i))-vy(i)*cos(psi(i));
%     y_dot(index)  =vx(i)*cos(psi(i))+vy(i)*sin(psi(i));
%     ax_veh(index) = ax(i);
%     ay_veh(index) = ay(i);
%     psi_dot_veh(index) = psi_dot(i);
% end

%% 
y_obs_log = zeros(length(ax),2);
for i = 1:length(ax)

 theta  = steering(i)/sw_ratio/180*3.14;
 y_mes = [ay(i);psi_dot(i)];
 vx_in = vx(i);
 [y_obs] = vy_direct_fcn(vx_in, y_mes, theta) ;
 
 y_obs_log(i,:) = y_obs';
 
end
vy_obs = y_obs_log(:,1);
  %%
  
  plot(vy_obs);hold on
  plot(vy); % lateral velocity from the GPS process data
  legend('Direct Obs','Carsim Real')
  title('MKZ Carsim data @ 90km/h')
  ylabel('Lateral Velocity [m/s]')
  xlabel('Index[-]')
  

