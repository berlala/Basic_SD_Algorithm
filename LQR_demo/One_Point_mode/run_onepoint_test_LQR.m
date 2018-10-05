%% Unit Test for SPS lateral control block
%% FF BolinZHAO bolin.zhao@ff.com
%% back ground clear
 clc;clear;close all;
 warning off
 %addpath(genpath('way point samples'))
 %% Path 
Cur  = pwd;
tips = regexp(Cur,'\');
%%
% 
%  PREFORMATTED
%  TEXT
% 
root = Cur(1:tips(4));
addpath([root,'Mfiles\Params']);
%loadMapFFpark_SPSonly;
addpath([root,'MfilesSPS']);
addpath([Cur,'\way_points_samples']);

 %% Config
Ts      = 0.1; % For Simulink
v        = 1; %m/s

x_init    = 0;
y_init    = 0;
psi_init  = pi/2;

Target_point = [20,20,pi/6];
%%

% Spiral_Msg = double(Spiral_Msg); % Force DataType change
% x = Spiral_Msg(:,1);
% y = Spiral_Msg(:,2);
% psi =  Spiral_Msg(:,3);
% RouteMapArray = [x,y,psi];
% K =  Spiral_Msg(:,4);
% CurvDerVehMap  = K;
% dis =  Spiral_Msg(:,5);  
%T_all =  double(ceil(max(dis)/v));

% figure(1)
% s(1) = subplot(3,1,1);
% plot(x,y);
% xlabel('Position X');ylabel('Position Y')
% subplot(3,1,2)
% plot(psi/pi*180)
% ylabel('Heading [Deg]')
% subplot(3,1,3)
% plot(K)
% ylabel( 'Curature []')
% title(s(1), 'Input info from Local Planner')
% subplot(4,1,4)
% plot(dis)
% ylabel( 'Traverl Distance [m]')

%% Simulation

%initial_ENH = [x_init,y_init,psi_init];

SteerCmd_h = [];x_real = [];y_real = [];psi_real = [];delta_log = [];
pe = 0; pth_e = 0; delta_f = 0;
x_c =x_init;
y_c =y_init;
psi_c = psi_init;

hwait=waitbar(0,'In Process>>>>>>>>');
time = 2000;



for i = 1:time
 
waitbar(i/time,hwait,'In Process>>>>');
 %[delta,e, th_e] = LQRfcn_apollo(x_c,y_c,psi_c, RouteMapArray, CurvDerVehMap, v, pe, pth_e) ;  
[delta,e, th_e] = LQRfcn_ff_one(x_c,y_c,psi_c, Target_point, v, pe, pth_e) ;  
 
 delta_f = delta;
%%%%Discrete Model Simulation%%%
x_c   = x_c + v * cos(psi_c) * Ts;
y_c   = y_c + v * sin(psi_c) * Ts;
psi_c = psi_c + v /(3)*tan(delta_f) * Ts;

%%%%Simuink Simulation%%%%%
%   sim LQR_fcn_test.slx; % ONLY Simulation for length Ts
% x_c =x_state(end);
% y_c =y_state(end);
% psi_c =psi_state(end);
% %%%%%%%%%%
pe = e;
pth_e = th_e;

x_real = [x_real;x_c];
y_real = [y_real;y_c];
psi_real = [psi_real;psi_c];
delta_log = [delta_log;delta];

end
close(hwait)


figure(2)
plot(x_real,y_real); hold on
plot(Target_point(1),Target_point(2),'o')
legend('Vehicle','Target')
xlabel('X');ylabel('Y');

figure(3)
plot(psi_real/pi*180)
hold on
plot(delta_log/pi*180)
legend('Heading','Steering Cmd')

% P1 = 1:1:length(psi);
% P2 = 1:1:length(psi_state);
% P3 = length(P1)/length(P2)*P2; % Re-Sample For P1 data
% Data_P3 = interp1(P1,psi,P3);
% Data_P1 =  [P1',psi];
% Data_P2 = [P2',psi_state];
% 
% figure(3)
% plot(Data_P2(:,2)/pi*180);
% hold on
% plot(Data_P3/pi*180);
% legend('Vehicle','Target')
% xlabel('Index[-]')
% ylabel('Heading[Deg]')

% figure(4)
% plot(beta_cmd/pi*180)
% hold on
% plot(beta_rep/pi*180)
% legend('Desired Front Wheel Steering[deg]','Response Front Wheel [deg]')
% xlabel('Index[-]')
% ylabel('Front Steering Angle[deg]')
%% Evaluation of the Vehicle Model 
    % ref to result_analysis.m
%% History Code
% figure(2)
% pis_str =  ([SteerCmd_h, zeros(1,10)]/180*pi)+psi' ; %+
% u = 1*sin(pis_str);
% v = 1*cos(pis_str);
% quiver(x_real,y_real,0.01*v(1:length(x_real)),0.01*u(1:length(x_real)));
% hold on
% plot(x_real, y_real,'o')
% hold on
% plot(x,y, '*-')
% 
% % for i = 2:length(x) 
% %     %num2str(pis_str(i))
% %     text(double(x(i)), double(y(i)), num2str(pis_str(i)));
% %     %added by Mr.Gao Ye
% % end
% title('SteeringWheel Angle and Vehicle Position')
% 
% figure(3)
% plot(SteerCmd_h)
% ylabel('SteerWheel [Deg]')

%% Test Mark
%[WP_UT_02] i = 96 to 97, SteeringWheel dramatic change 
%i [96]  = [19.1519, 3.8373,0.5923] 
% i [97] = [19.3244, 3.9549, 0.6045]  % change SteerCmd(5) to SteerCmd(end)

%i[227] (19.49,26.8391,2.5195)
%i[228](19.3195, 26.9599,2.5309)