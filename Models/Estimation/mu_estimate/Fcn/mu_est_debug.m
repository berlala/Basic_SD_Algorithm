% Function for Mu Estimate
%%
function [mu,vy_obs,Fyf_est,Fyr_est,alpha_f_est,alpha_r_est,Fyf_offline_est,Fyr_offline_est,data_saturation_f_est,data_saturation_r_est]  = mu_est_debug(fz, vx, ay, psi_dot, steering)
%%
if mean(vx)> 50/3.6
slip_check = 3;
deadzone_down  = 1200;
deadzone_up = 1400;
else
slip_check = 0.5;
deadzone_down  = 400;
deadzone_up = 800;
end
%% MKZ Parameters from CarSim, From Yonglong Zhang
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

%% 0. Data Process
yaw_rate_dot = [0;diff(psi_dot)]*fz;
%% 1. Velocity Estimation 
y_obs_log = zeros(length(ay),2);
for i = 1:length(ay)

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

%%  Slip Angle 
% Calculate The Slip Angle
eps = 0.0000001;

for i = 1:length(vy_obs)
theta  = steering(i)/sw_ratio/180*pi; %[rad]
alpha_f_est(i) = -theta - atan((-vy_obs(i) - lf*psi_dot(i))/(vx(i)+eps)); %[rad]
alpha_r_est(i) = atan((vy_obs(i) - lr*psi_dot(i))/(vx(i)+eps));
end

%%   Est Data
 Fyf_offline_est = Cf_axle*-alpha_f_est;
 Fyr_offline_est = Cr_axle*-alpha_r_est;

 data_saturation_f_est = [];
 data_saturation_r_est = [];

for i = 2:length(vx)
  if (abs(Fyf_offline_est(i)) - deadzone_down)>abs(Fyf_est(i)) && (abs(Fyf_est(i))>400) && ((abs(Fyf_offline_est(i)) <(abs(Fyf_est(i))+deadzone_up)))  && (sign(Fyf_offline_est(i))==sign(Fyf_est(i)))...
                    &&(abs(Fyf_offline_est(i)) > abs(Fyf_offline_est(i-1))) && (abs(alpha_f_est(i))>slip_check/180*pi) 
      %  disp('Detect Saturation!')
        data_saturation_f_est = [data_saturation_f_est;[i,alpha_f_est(i), Fyf_offline_est(i), Fyf_est(i)]];%[index, alpha_angle,Fy_offline_est, Fy_realtime_est]
    else
    end  
end

for i = 2:length(vx)
  if ((abs(Fyr_offline_est(i)) - deadzone_down)>abs(Fyr_est(i))) && (abs(Fyr_est(i))>400) && ((abs(Fyr_offline_est(i)) <(abs(Fyr_est(i))+deadzone_up)))&& (sign(Fyr_offline_est(i))==sign(Fyr_est(i)))...
          &&(abs(Fyr_offline_est(i)) > abs(Fyr_offline_est(i-1))) && (abs(alpha_r_est(i))>slip_check/180*pi) 
      %  disp('Detect Saturation!')
        data_saturation_r_est = [data_saturation_r_est;[i,alpha_r_est(i), Fyr_offline_est(i), Fyr_est(i)]];
    else
    end  
end
%% 
if ~isempty(data_saturation_f_est)
    mu_est_mean_linear_f = mean(abs(data_saturation_f_est(:,3))/(mf*9.8));
else
    mu_est_mean_linear_f = 0;
end

if ~isempty(data_saturation_r_est)
    mu_est_mean_linear_r = mean(abs(data_saturation_r_est(:,3))/(mr*9.8));
else
    mu_est_mean_linear_r =0;
end

mu = round(max(mu_est_mean_linear_f, mu_est_mean_linear_r), 2);
%%

end