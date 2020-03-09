clear;clc
load carsim_data_cs2.mat;
%% Config
Ts = time_2(2);
clear theta_all


str_ratio = 22.5;
% The following parameters need to be update 
theta_all = SWA_10/180*pi/22.5;
% Initial States
v_x =10; % change velocity here
v_y = 0;
yaw_rate =0;
x = x_10(2);
y = y_10(2); 
phi = phi_10(1)/180*pi;

vx_0 = v_x;
x_0 = x;
y_0 = y;
phi_0 = phi;
%%
%[x_dot], [y_dot] in vehicle body frame 
g = 9.81;
m = 2204;
F_z_1 = m*g/4;
F_z_2 = m*g/4;
F_z_3 = m*g/4;
F_z_4 = m*g/4;
 lf = 1.3472; 
 lr = 1.5028;
W_a = 1.8; % 
Iz = 4220;

%% Test Input 
%theta = 0/180*pi; % Steering Angle[rad], input command
%theta_FL = theta;
%theta_FR = theta;
theta_RL =  0;
theta_RR =  0;

V_tire_y_1 = 0;
V_tire_y_2 = 0;
V_tire_y_3 = 0;
V_tire_y_4 = 0;

%To-do: update the kappa calculation from chassis frame
kappa_1 = 0.00;
kappa_2 = 0.00;
kappa_3 = 0.00;
kappa_4 = 0.00;

%%
for i = 1:length(theta_all)
theta  = theta_all(i);
%% Ackerman Steering on Front Wheels
[theta_FL,theta_FR] = ackerman_steering(theta);
%% Tire Force in Tire frame
% [V_tire_x] is the tire frame X speed
%kappa is the key to drive the car forward
[f_x_FL] = brush_tyre_long_TS(F_z_1, kappa_1); % V_sx  = V_tire_x - V_r, V_r = Omega*R; Kappa could be calculated from wheel sensor and imu
[f_y_FL,M_z_FL,~] = brush_tyre_lateral_TS(F_z_1,v_x, v_y, yaw_rate, theta_FL, 1);
    
[f_x_FR] = brush_tyre_long_TS(F_z_2, kappa_2); 
[f_y_FR,M_z_FR,~] = brush_tyre_lateral_TS(F_z_2,v_x, v_y, yaw_rate, theta_FR, 2);
        
[f_x_RL] = brush_tyre_long_TS(F_z_3, kappa_3);
[f_y_RL,M_z_RL,~] = brush_tyre_lateral_TS(F_z_3,v_x, v_y, yaw_rate, 0, 3);
        
[f_x_RR] = brush_tyre_long_TS(F_z_3, kappa_4); 
[f_y_RR,M_z_RR,~] =brush_tyre_lateral_TS(F_z_4,v_x, v_y, yaw_rate, 0, 4);

%% Tire Force in Vehicle frame
%FL
F_x_FL = f_x_FL*cos(theta_FL) - (-f_y_FL)*sin(theta_FL);
F_y_FL =  f_x_FL*sin(theta_FL) + (-f_y_FL)*cos(theta_FL);
%FR
F_x_FR = f_x_FR*cos(theta_FR) - (-f_y_FR)*sin(theta_FR);
F_y_FR =  f_x_FR*sin(theta_FR) + (-f_y_FR)*cos(theta_FR);
%RL
F_x_RL = f_x_RL*cos(theta_RL) - (-f_y_RL)*sin(theta_RL);
F_y_RL =  f_x_RL*sin(theta_RL) + (-f_y_RL)*cos(theta_RL);
%RR
F_x_RR = f_x_RR*cos(theta_RR) - (-f_y_RR)*sin(theta_RR);
F_y_RR =  f_x_RR*sin(theta_RR) + (-f_y_RR)*cos(theta_RR);
%% Overall Vehicle in GLB frame
% in VEH Frame
a_x = (m*v_y*yaw_rate + F_x_FL + F_x_FR + F_x_RL + F_x_RR)/m; 
a_y = (-m*v_x*yaw_rate +  F_y_FL + F_y_FR + F_y_RL + F_y_RR)/m;
yaw_rate_dot = (lf*(F_y_FL + F_y_FR) - lr*(F_y_RL + F_y_RR)  + W_a/2*(-F_x_FL + F_x_FR - F_x_RL + F_x_RR))/Iz;

v_x  = v_x  +(a_x)*Ts; 
v_y = v_y + (a_y)*Ts;
yaw_rate = yaw_rate + yaw_rate_dot*Ts;
% in GLB Frame
x   = x + (v_x * cos(phi) - v_y*sin(phi) )* Ts;
y   = y +  (v_x* sin(phi) + v_y*cos(phi) )* Ts;
phi = phi + yaw_rate*Ts;

x_log(i) = x;
y_log(i) =y;
phi_log(i) = phi;


v(i) = sqrt(v_x^2 + v_y^2);
f_y_FL_log(i) = f_y_FL;
f_y_RR_log(i) = f_y_RR;
end

%% Review
time = (1:i)*Ts;
figure(1)
plot(x_log, y_log,'.');hold on
figure(2)
plot(time, phi_log/pi*180);
ylabel('Heading[deg]')
figure(3)
plot(time, v)
ylabel('speed[m/s]')
figure(4)
subplot(2,1,1)
plot(time, f_y_FL_log)
ylabel('lateral force front left tire')
subplot(2,1,2)
plot(time, f_y_RR_log)
ylabel('lateral force rear right tire')