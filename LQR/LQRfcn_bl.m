function [delta,e,th_e,ind,tgt] = LQRfcn_bl(initial_ENH, RouteMapArray, CurvDerVehMap, v, pe, pth_e, p_delta ,Gain_f,Gain_fb,Preview)
% Bolin ZHAO, ff china, bolin.zhao@ff.com
% This fcn is for the out of Simulink simulation.
% n =3, fixed control horizon
% 2019/7/5: update the lqr solver
%%
% e is the position error
% th_e is the heading error
%% Default Paramters
lr = 1.5;
lf = 1.7;
Iz = 3600;
dt = 0.02; % sample time
Cf = 36600;
Cr = Cf;
m  = 2300;
L = 3;% wheelbase

x_c = initial_ENH(1);
y_c = initial_ENH(2);
psi_c = initial_ENH(3);

%% Config

%Preview  =1; % [horizonX, to be updated]

Heading = (psi_c )*180/pi;
RotMat = [cosd(Heading), sind(Heading); ...
                   -sind(Heading), cosd(Heading)];
      
[ind,~,X_clst] = find_closest(RouteMapArray,initial_ENH);      
tgt = min(ind+Preview, length(RouteMapArray(:,1)));

x_tg = RouteMapArray(tgt,1);
y_tg = RouteMapArray(tgt,2);
psi_tg= RouteMapArray(tgt,3);

pt0lc = RotMat*[X_clst(1)-x_c,...
                X_clst(2)-y_c]';
pt1lc = RotMat*[x_tg-x_c,...
               y_tg-y_c]';

e =0 -pt1lc(2);
th_e = initial_ENH(3)- psi_tg; % Heading error [rad]

%e = df*-sign_stan; % Without [-sign_stan], it may cannot strict follow
X = zeros(4, 1);%# State  [Position_Error, Position_Error_dot, Heading_error, Heading_error_dot]
X(1, 1) =  e;   
X(2, 1) =(e - pe) / dt;
X(3, 1) =  th_e;
X(4, 1) = (th_e - pth_e) / dt;

%%
k = CurvDerVehMap(tgt);

%A, B realte to Vehicle Model
% X = [Position_Error, Position_Error_dot, Heading_error, Heading_error_dot]
%U = [delat_f]

% 
Ac = [0, 1, 0, 0;
     0, -(2*Cf+2*Cr)/(m*v), (2*Cf+2*Cr)/m, (-2*Cf*lf + 2*Cr*lr)/(m*v);
     0, 0, 0 ,1;
     0, -(2*Cf*lf-2*Cr*lr)/(Iz*v), (2*Cf*lf-2*Cr*lr)/Iz, (-2*Cf*lf^2-2*Cr*lr^2)/(Iz*v)];
Bc = [0 2*Cf/m, 0, 2*Cf*lf/Iz]'; 
A = zeros(4,4);
B = zeros(4,1);
[A,B] = c2d(Ac,Bc,dt);

% A = [1 T 0 0;
%      0 1-64*T/v 64*T -6.4*T/v;
%      0 0 1 T;
%      0 -4*T/v 4*T 1-13*T/v];
%  B = [0 0.32 0 0.35]';

%  A = [1 T 0 0;
%      0 1-64*T/v 0 -6.4*T/v;
%      0 0 1 T;
%      0 -4*T/v 0 1-13*T/v];
%  B = [0 0.32 0 0.35]';
%  

Q = eye(4);
Q(1,1) =1; %big value lead failure [To be explored]: This value should be related to the initial error...
%... try sqrt(1/Q(1,1)) = max_initial_error % Key for the overshoot
Q(2,2) = 1;
Q(3,3) = 1;
Q(4,4) = 1;

R = 0.1*eye(1);% around 3, reasonable value for fluclation in the steering control cmd 

%coder.extrinsic('dlqr'); 
%% Calculate for u(k)
%1)  Method 1
% [U_lqr]= solver_dlqr_tue(A,B,Q,R,X);
% fb = U_lqr;

%  Method 2
% Only For Off-line Simulation
% [P,~,~] = dare(A,B,Q,R);
% K_lqr = -(B'*P*B +R )^-1*B'*P*A;
% fb = K_lqr *X;

% % Method 3
%For off-line Simulation
[K_lqr,~,~] = dlqr(A,B,Q,R,0);
fb = -K_lqr *X;

% Method 4
% [U_lqr] = solver_dlqr(A,B,Q,R,X);
% fb = U_lqr;
%%
%Curvature FeedForward  + LQR Feedback
ff = atan2(L * k, 1) ;  %[rad]
%fb = -K_lqr *X;  %[rad]
%fb_a = rem((fb) ,(2*pi))-2*pi ;

%P_ff = 0.01;
delta = Gain_f*ff + Gain_fb*fb;

%% Constrain Check 
if  delta > 45/180*pi
    delta = 45/180*pi;
elseif delta < -45/180*pi
       delta = -45/180*pi;
end

if isnan(delta)
    delta =0;
end

e_delta = delta - p_delta;
limit = 30/180*pi; % front wheel change rate , n Deg/second
Limit_sample = limit*dt;
if abs(e_delta) > Limit_sample
    delta = p_delta + sign(e_delta)*Limit_sample;
end
return
