%% CF Estimate
%%
% bolinzhao@yahoo.com
% This scripte is for CF estimation.
%% Load Data
close all;
clear;   % 1  for left
%NEOLIX
% chassis = csvread('cf_est_chs_Cf_est_1.csv', 2 ,0);
% pose = csvread('cf_est_pos_Cf_est_1.csv', 2 ,0);
% sw_max = 30 ; %[deg], max front wheel steering angle
% a = 1;  % imu to front axle 
% b = 0.76; %  imu to front axle
% m_f = 560/2 ;
% m_r = 560/2;

%MKZ
chassis = csvread('cf_est_chs_caca.csv', 2 ,0); %MKZ data
pose = csvread('cf_est_pos_caca.csv', 2 ,0);
%related parameters
sw_max = 29 ; %[deg], max front wheel steering angle
a = 2.84;  % imu to front axle 
b = 0; %  imu to front axle
m_f = 580*2 ;
m_r = 580*2;
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
disp(' All Data have been loaded.')

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
end
%
figure(1)
plot(y_dot); % notice positive y_dot is the headlamp direction in Mid-Car
hold on
plot(v)
legend('X_{dot}','V_{chassis}')
ylabel('Speed[m/s]')
xlabel('Index[-]')
%%  Calculate the Axle Force and Slip Angle
%35[deg] for maximum front wheel 
m = m_f + m_r; 
If = m_r/m*(a+b);  % to be checked
Ir = m_f/m*(a+b); 
Iz = ((a+b)*(m_f/m))^2*m_f + ((a+b)*(m_r/m))^2*m_r;

% Calcualte the Psi_ddot
psi_ddot =  [0; diff(psi_dot)]./([0.01]);
% psi_ddot = [0; gradient(psi_dot)];

% for i= 1:length(T_pose_ind)
%     F_xr(i) = (Iz*psi_ddot(T_pose_ind(i))+m*If*(ax(T_pose_ind(i))-psi_dot(T_pose_ind(i))*y_dot(i)))/(If+Ir);
%     F_xf(i) = m*(ax(T_pose_ind(i))-psi_dot(T_pose_ind(i))*y_dot(i)) - F_xr(i);
% end

for i= 1:length(T_pose_ind)
    F_xf(i) = (Iz*psi_ddot(T_pose_ind(i))-m*Ir*(ax(T_pose_ind(i))-psi_dot(T_pose_ind(i))*y_dot(i)))/((If+Ir)*-cos(-steering(i)/100*sw_max/180*pi ));
    F_xr(i) = m*(ax(T_pose_ind(i))-psi_dot(T_pose_ind(i))*y_dot(i)) - F_xf(i)*cos(-steering(i)/100*sw_max/180*pi );
end

% Calculate The Slip Angle
eps = 0.0000001;

for i = 1:length(x_dot)
alpha_f(i) = -steering(i)/100*sw_max/180*pi - atan((x_dot(i) - a*psi_dot(T_pose_ind(i)))/(y_dot(i)+eps));
alpha_r(i) = -atan((x_dot(i) + b*psi_dot(T_pose_ind(i)))/(y_dot(i)+eps));
end
figure(2)
subplot(2,1,1)
plot(alpha_f, F_xf,'.'); hold on
subplot(2,1,2)
plot(alpha_r, F_xr,'.'); hold on
xlabel('slip angle[rad]')
ylabel('lateral force [N]')
%% Calulate the Stiffness
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
%% Rear
K_data = [alpha_r;F_xr]';
 [Idx,C]=kmeans(K_data,20);
figure(2)
plot(C(:,1), C(:,2),'o','linewidth',3);hold on
 pf = polyfit(C(:,1), C(:,2), 1);
 aa = [-0.2:0.01:0.2];
 bb  = pf(1)*aa + pf(2);
 disp([pf(1)])
 plot(aa , bb,'linewidth',3)
 %% Front
 K_data = [alpha_f;F_xf]';
 [Idx,C]=kmeans(K_data,20);
figure(2)
plot(C(:,1), C(:,2),'o','linewidth',3);hold on
 pf = polyfit(C(:,1), C(:,2), 1);
 aa = [-0.2:0.01:0.2];
 bb  = pf(1)*aa + pf(2);
 disp(pf(1))
 plot(aa , bb,'linewidth',3)
  legend('Rear','Rear fit line','Front','Front fit line')
  xlabel('Slip Angle[rad]')
 ylabel('Lateral Force[N]')
 