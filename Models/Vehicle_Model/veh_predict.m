function [x_log,y_log,psi_log, v_x_log, v_y_log, yaw_rate_log] = veh_predict(theta , v_x, v_y, yaw_rate, x, y, psi, horizon)
% zhaobolin
% this function is based on a chassis model and brush tyre model to predict
% the future vehicle position states. 
% To be Noticed: the longitude speed is assumed as constant currently.
% 
% [theta] is the front wheel steering angle in [rad]


%%
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

Ts = 1/50; % the predict time is Ts * horizon

theta_FL = theta;
theta_FR = theta;
theta_RL =  0;
theta_RR =  0;

%To-do: update the kappa calculation from chassis frame
kappa_1 = 0.00;
kappa_2 = 0.00;
kappa_3 = 0.00;
kappa_4 = 0.00;

%%
for i = 1:horizon
%% Tire Force in Tire frame
% [V_tire_x] is the tire frame X speed
%kappa is the key to drive the car forward
[f_x_FL] = brush_tyre_long_TS(F_z_1, kappa_1); % V_sx  = V_tire_x - V_r, V_r = Omega*R; Kappa could be calculated from wheel sensor and imu
[f_y_FL,M_z_FL,~] = brush_tyre_lateral_TS(F_z_1,v_x, v_y, yaw_rate, theta, 1);
    
[f_x_FR] = brush_tyre_long_TS(F_z_2, kappa_2); 
[f_y_FR,M_z_FR,~] = brush_tyre_lateral_TS(F_z_2,v_x, v_y, yaw_rate, theta, 2);
        
[f_x_RL] = brush_tyre_long_TS(F_z_3, kappa_3);
[f_y_RL,M_z_RL,~] = brush_tyre_lateral_TS(F_z_3,v_x, v_y, yaw_rate, theta, 3);
        
[f_x_RR] = brush_tyre_long_TS(F_z_3, kappa_4); 
[f_y_RR,M_z_RR,~] =brush_tyre_lateral_TS(F_z_4,v_x, v_y, yaw_rate, theta, 4);

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
x   = x + (v_x * cos(psi) - v_y*sin(psi) )* Ts;
y   = y +  (v_x* sin(psi) + v_y*cos(psi) )* Ts;
psi = psi + yaw_rate*Ts;

x_log(i) = x;
y_log(i) =y;
psi_log(i) = psi;

v_x_log(i) = v_x;
v_y_log(i) = v_y;
yaw_rate_log(i) = yaw_rate;

v(i) = sqrt(v_x^2 + v_y^2);
%f_y_FL_log(i) = f_y_FL;
%f_y_RR_log(i) = f_y_RR;

end



end