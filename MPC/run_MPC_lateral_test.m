%% FF BolinZHAO bolin.zhao@ff.com
%% back ground clear
 clc;clear;close all;
 warning off;
 %addpath(genpath('way point samples'))
 %% Path 
Cur  = pwd;
tips = regexp(Cur,'\');
%% 
% root = Cur(1:tips(4));
% addpath([root,'Mfiles\Params']);
% %loadMapFFpark_SPSonly;
% addpath([root,'MfilesSPS']);
% addpath([Cur,'\way_points_samples']);

 %% Config
Ts      = 0.05; % For Plant Update
v        = 10; %m/s
display(['Simulation velocity is =', num2str(v*3.6), 'Km/h'] )

x_init    = 0;
y_init    = 0;
psi_init  =0/180*pi;

Gain_f = 0.4; % This value need re-calibrate on the Real Car
Gain_fb =0.8; % This value need re-calibrate on the Real Car
Preview = v*1/0.2;

lf =1.7;
L =3.2;

% x_init    = 0;
% y_init    = -2;
% psi_init  = 0.1;
% 
% Gain_f = 0.1;
% Preview = 5;
%%
load WP_LT_02;
%load WP_UT_02;
%load WP_LC_02;
%load RouteMap_single;
%load S_Line.mat;
%load I_Line.mat;

Spiral_Msg = double(Spiral_Msg); % Force DataType change
x = Spiral_Msg(:,1);
y = Spiral_Msg(:,2);
psi =  Spiral_Msg(:,3);
RouteMapArray = [x,y,psi];
K =  Spiral_Msg(:,4);
CurvDerVehMap  = K;
dis =  Spiral_Msg(:,5);  
%T_all =  double(ceil(max(dis)/v));

figure(2)
plot(x,y,'-');hold on
legend('Vehicle','Target')
xlabel('X');ylabel('Y');
%% Simulation

%initial_ENH = [x_init,y_init,psi_init];

SteerCmd_h = [];x_real = [];y_real = [];psi_real = [];index_target_log = [];
pe = 0; pth_e = 0; delta_f= 0;e_log = [];
x_c =x_init;
y_c =y_init;
psi_c = psi_init;
LOG_log = {};
delta_log = zeros(1,9);

hwait=waitbar(0,'In Process>>>>>>>>');
time = floor(dis(end)/v/Ts);

for i = 1:time
 initial_ENH = [x_c, y_c, psi_c];
waitbar(i/time,hwait,'In Process>>>>');
 %[delta,e, th_e] = LQRfcn_apollo(x_c,y_c,psi_c, RouteMapArray, CurvDerVehMap, v, pe, pth_e) ;  
 %[delta,e, th_e,index_target] = LQRfcn_ff(x_c,y_c,psi_c, RouteMapArray, CurvDerVehMap, v, pe, pth_e) ;
% [delta,e, th_e,index_target] = LQRfcn_guan(x_c,y_c,psi_c, RouteMapArray, CurvDerVehMap, v, pe, pth_e) ;  
 [delta,e, th_e,~,index_target,LOG] = MPC_bl1(initial_ENH,v, RouteMapArray, CurvDerVehMap, pe, pth_e,Preview,delta_f); 
 % e is heading error
 %th_e is lateral error
 
%if delta< 0.01&& delta>-0.01
%    delta_f = 0;
%else
if isnan(delta)
    delta_f = delta_f;
elseif delta >45/180*pi
    delta_f = 45/180*pi;
elseif delta < -45/180*pi
    delta_f = -45/180*pi;
else
delta_f = delta;
end

%%%%%%% Calibration %%%%%%%
G_fb =1;
delta_f = G_fb * delta_f;

%%%%%%% Discrete Low Pass Filter%%%%%%%
delta_f_f  = delta_f;
%delta_f_f =delta_f*0.2 + delta_log(end)*0.2+delta_log(end-1)*0.2+ delta_log(end-2)*0.2+delta_log(end-3)*0.1+...
                  % delta_log(end-4)*0.1+delta_log(end-5)*0.1;

%%%%REAL WORLD STATE UPDATE Simulation%%%
x_c   = x_c + v * cos(psi_c) * Ts; %global position
y_c   = y_c + v * sin(psi_c) * Ts;
psi_c = psi_c + v /(lf)*tan(delta_f_f) * Ts;  % global heading

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
e_log = [e_log;pe];
psi_real = [psi_real;psi_c];
delta_log = [delta_log,delta_f_f];
index_target_log = [index_target_log;index_target];
LOG_log{i,1} = {LOG};

figure(2)
plot(x_c,y_c,'o','linewidth',1); 

figure(4)
plot(i,delta_f_f,'o','linewidth',1); hold on

end
close(hwait)

figure(3)
plot(Ts*(1:length(psi_real)), psi_real/pi*180)
hold on
plot(Ts*(1:length(delta_log)),delta_log/pi*180)
plot(Ts*(1:length(delta_log)),delta_log*12/pi*180)
% hold on
% plot(index_target_log)
legend('Response Heading[deg]','Front Axle Str Angle[deg]','SWA Cmd[deg]')

display(['the biggest tracking error is ', num2str(max(abs(e_log))),' meter'])
