%% Mu Estimate 
% In this file, the Vy, Lateral Force and Yaw-rate dot are directly from the Carsim
% dataset to prove the mu-estimate method is correct.
%% 1. Load Data
close all;
clear;   % 1  for left
% Detection Windows Setting
win_period = 0.5;%[s]
win_shift = 0.1;

%MKZ
%Time,Ax,Ay,Vx,Vy,Yaw,Steer_SW,AVz,X_S1,Y_S1

% Load the history data/logger
%data_all = csvread("50kmh0307MKZ0323.csv",2,0);mu_true =data_all(:,26);
data_all = csvread("50kmh0703MKZ0323.csv",2,0);  mu_true =data_all(:,26);
%data_all = csvread("60kmh04085MKZ0323.csv",2,0); 
%data_all = csvread("60kmh08504MKZ0323.csv",2,0);
%data_all = csvread("70kmh0307MKZ0323.csv",2,0);
%data_all = csvread("70kmh0703MKZ0323.csv",2,0);
%data_all = csvread("90kmh04085MKZ0323.csv",2,0);
%data_all = csvread("90kmh08504MKZ0323.csv",2,0);
%%
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
shift_length = win_shift/0.01;
win_length = win_period/0.01;
mu_logger  =[0.85];
t_logger = [0];
Ts = data_all(1,1); fz = 1/Ts;
T_all = data_all(:,1);
for index = 1:shift_length:length(data_all)
    %Fake data input collection
    clear data;
    if (index+win_length)>length(data_all)
        index_end = length(data_all);
    else
        index_end = index+win_length;
    end
    data = data_all(index:index_end,:);
    %ISO
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
    
    %% 2. Mu Estimate
    [mu]  = mu_est(fz, vx, ay, psi_dot, steering);
    
    %% 3.Outside Logic (in further move into Mu_est with history mu)
    if mu == 0&&~isempty(mu_logger)
        mu = min(mu_logger(end),0.85);
    end
    mu_logger = [mu_logger,mu];
    t_logger = [t_logger,index*Ts];
end

plot(t_logger,mu_logger);hold on
%plot(T_all, mu_true);
%legend('Estimation','True')
xlabel('Time[s]')
ylabel('mu[-]')
