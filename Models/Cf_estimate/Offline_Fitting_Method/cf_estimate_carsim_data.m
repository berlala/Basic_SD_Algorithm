%% CF Estimate
%%
% bolinzhao@yahoo.com
% This scripte is for CF estimation.
%% Load Data
close all;
clear;   % 1  for left
% Tire Real front : 5e4, Rear: 

% CarSim MKZ
data = csvread('40kmh_MKZ0316.csv',2,0);
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
theta  = steering/sw_ratio/180*3.14;
Ts = data(1,1); fz = 1/Ts;
psi_ddot = [0;diff(psi_dot)]*fz;
disp(' All Data have been loaded.')

%%  Calculate the Axle Force and Slip Angle
% Calcualte the Psi_ddot

% for i= 1:length(T_pose_ind)
%     F_xr(i) = (Iz*psi_ddot(T_pose_ind(i))+m*lf*(ax(T_pose_ind(i))-psi_dot(T_pose_ind(i))*y_dot(i)))/(lf+lr);
%     F_xf(i) = m*(ax(T_pose_ind(i))-psi_dot(T_pose_ind(i))*y_dot(i)) - F_xr(i);
% end

%with psi_dot
% for i= 1:length(steering)
%     F_xf(i) = (Iz*psi_ddot(i)-m*lr*(ay(i)-psi_dot(i)*vy(i)))/(ll*-cos(-theta(i)));
%     F_xr(i) = m*(ay(i)-psi_dot(i)*vy(i)) - F_xf(i)*cos(-theta(i) );
% end

%without psi_dot
for i = 1:length(ay)
F_xf(i) = (m*lr/ll * ay(i) + Iz/ll * psi_ddot(i))/cos(theta(i));
F_xr(i) = m*lf/ll*ay(i)  - Iz/ll*psi_ddot(i);
end

% Calculate The Slip Angle
eps = 0.00000001;

for i = 1:length(vy)
theta  = steering(i)/sw_ratio/180*pi; %[rad]
alpha_f(i) = -theta - atan((-vy(i) - lf*psi_dot(i))/(vx(i)+eps)); %[rad]
alpha_r(i) = atan((vy(i) - lr*psi_dot(i))/(vx(i)+eps));
end


figure(2)
subplot(2,1,1)
plot(alpha_f, F_xf,'.'); hold on
ylabel('Front lateral force [N]')
subplot(2,1,2)
plot(alpha_r, F_xr,'.'); hold on
xlabel('slip angle[rad]')
ylabel('Rear lateral force [N]')
%% Calulate the Stlffness
for i= 1:length(F_xr)
Cf(i) = F_xf(i)/(alpha_f(i)/pi*180);
Cr(i) = F_xr(i)/(alpha_r(i)/pi*180);
end

% figure(3)
% subplot(2,1,1)
% plot(Cf,'.');
% subplot(2,1,2)
% plot(Cr,'.')

Cf_mean = mean(Cf);
Cr_mean = mean(Cr);
 %% Front Axle
 K_data = [alpha_f;F_xf]';
 [Idx,C]=kmeans(K_data,10);
 figure(2)
subplot(2,1,1)
plot(C(:,1), C(:,2),'o','linewidth',3);hold on
 pf = polyfit(C(:,1), C(:,2), 1);
 aa = [-0.1:0.01:0.1];
 bb  = pf(1)*aa + pf(2);
 disp(pf(1))
 plot(aa , bb,'linewidth',3)
  legend('Front','Front fit line')
  xlabel('Slip Angle[rad]')
 ylabel('Lateral Force[N]')
%% Rear Axle
K_data_r = [alpha_r;F_xr]';
 [Idx,C_r]=kmeans(K_data_r,10);
 figure(2)
subplot(2,1,2)
plot(C_r(:,1), C_r(:,2),'o','linewidth',3);hold on
 pf = polyfit(C_r(:,1), C_r(:,2), 1);
 aa = [-0.1:0.01:0.1];
 bb  = pf(1)*aa + pf(2);
 disp([pf(1)])
 plot(aa , bb,'linewidth',3)
  legend('Rear','Rear fit line')
 