%% Load Data
close all;
clear;   % 1  for left

%MKZ
chassis = csvread('cf_est_chs_caca.csv', 2 ,0); %MKZ data
pose = csvread('cf_est_pos_caca.csv', 2 ,0);
%related parameters
sw_max = 29 ; %[deg], max front wheel steering angle
a = 2.84;  % imu to front axle 
b = 0; %  imu to front axle

% Parameters
Cf = 1.0436e5;
Cr = 1.4889e5;
lf = 2.84;
lr = 0;
%%
%pose data
vy = pose(:,3); % in GLB, head-lamp direction
vx = pose(:,7);
psi = pose(:,1); 
ax = pose(:,4); % in VEH, right hand direction
ay = pose(:,5); % in VEH, head-lamp direction
T_pose = pose(:,2);
psi_dot = pose(:,6); % anticlockwise direction is positive
%chassis data
T_chs = chassis(:,1) ;
v = chassis(:,2);
steering = chassis(:,3); % [percentage]
disp('MKZ record All Data have been loaded.')
%% Time Sync between Pose and Chassis
% plot(T_pose);hold on
% plot(T_chs)
% T_pose is eailier than T_chs.
T_pose_ind = [];
for i = 1:length(T_chs)
    for ii = 1:length(T_pose)
        if ii+1>length(T_pose)
            ii = ii-1;
        end
        if (T_pose(ii)<=T_chs(i))&&(T_pose(ii+1)>T_chs(i))
            if abs(T_pose(ii) - T_chs(i)) <= abs(T_pose(ii+1) - T_chs(i))
            T_pose_ind = [T_pose_ind; ii];
            else
             T_pose_ind = [T_pose_ind; ii+1];
            end
        end
    end
end
if length(T_pose_ind) == length(T_chs)
    disp('Length Matched!')
end
%%

%Change the data from MAP Frame to VEH Frame (x_dot/y_dot are in vehicle frame)
for index = 1:length(T_pose_ind)
    i = T_pose_ind(index); 
    x_dot(index)  =vx(i)*sin(psi(i))-vy(i)*cos(psi(i));
    y_dot(index)  =vx(i)*cos(psi(i))+vy(i)*sin(psi(i));
    ax_veh(index) = ax(i);
    ay_veh(index) = ay(i);
    psi_dot_veh(index) = psi_dot(i);
end

%%
x_obs_log = zeros(length(ax_veh),2);
for i = 1:length(ax_veh)
 
 theta  = steering(i)/100*sw_max/180*pi ;
 y_mes = [ax_veh(i);psi_dot_veh(i)];
 vx = y_dot(i);
 [x_obs] = vy_direct_fcn(vx, y_mes, theta) ;
 
 x_obs_log(i,:) = x_obs';
 
end
  %%
  
  plot(x_obs_log(:,1));hold on
  plot(x_dot) % lateral velocity from the GPS process data
  legend('Direct Obs','IMU')
  title('MKZ data')
  ylabel('Lateral Velocity [m/s]')
  xlabel('Index[-]')
  

