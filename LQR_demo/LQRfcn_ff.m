function [delta,pe,th_e,ind] = LQRfcn_ff( x_c,y_c,psi_c, RouteMapArray, CurvDerVehMap, v, pe, pth_e)
% Bolin ZHAO, ff china, bolin.zhao@ff.com
% This fcn is for the out of Simulink simulation.
%%
% e is the position error
% th_e is the heading error
%% Default Paramters
lr = 1.5;
lf = 1.7;
Iz = 3600;
dt = 0.1; % sample time
Cf = 36600;
Cr = Cf;
m  = 2300;
%% Config

Preview  = 1; % bigger value will lead failure [to be explored]
L = 3;% wheelbase

%%
%cx  =x;
%cy = y;
%cyaw = psi;
%ck = CurvDerVehMap;
initial_ENH = [x_c, y_c, psi_c];  % current vehicle state

%[ind] = findinitwaypoints(initial_ENH, RouteMapArray);
[ind,df] = find_closest(RouteMapArray,initial_ENH);
% find iniital way point is related to [psi]
%Reference Point
tgt = min(ind+Preview, length(RouteMapArray(:,1)));
x_tg = RouteMapArray(tgt,1);
y_tg = RouteMapArray(tgt,2);

%x_tg = 30.1738;
%y_tg = 30.1601;

psi_tg= RouteMapArray(tgt,3);
%psi_tg = 1.5764;


k = CurvDerVehMap(tgt);

%e = sqrt((x_tg-x_c)^2 +(y_tg-y_c)^2);
th_e = initial_ENH(3)-psi_tg; % Heading error [rad]

%A, B realte to Vehicle Model
% X = [Position_Error, Position_Error_dot, Heading_error, Heading_error_dot]
%U = [delat_f]
Ac = [0, 1, 0, 0;
      0, -(2*Cf+2*Cr)/(m*v), (2*Cf+2*Cr)/m, (-2*Cf*lf + 2*Cr*lr)/(m*v);
      0, 0, 0 ,1;
      0, -(2*Cf*lf-2*Cr*lr)/(Iz*v), (2*Cf*lf-2*Cr*lr)/Iz, (-2*Cf*lf^2+2*Cr*lr^2)/(Iz*v)];
Bc = [0 2*Cf/m, 0, 2*Cf*lf/Iz]';
sys =ss(Ac, Bc,[],[]);
sysd = c2d(sys,dt);
A = sysd.a;
B = sysd.b;

Q = eye(4);
Q(1,1) = 5; %big value lead failure [To be explored]: This value should be related to the initial error...
%... try sqrt(1/Q(1,1)) = max_initial_error
Q(2,2) = 1;
Q(3,3) = 1;
Q(4,4) = 1;

R = eye(1);

%coder.extrinsic('dlqr');
[K_lqr, ~, ~ ]= dlqr(A, B, Q, R);
%K_lqr = [0.1,0.1,0.1,0.1];

% pe_1= e;
% pth_e_1 = th_e;
% Local Coorid
Heading = (initial_ENH(3) )*180/pi;
RotMat = [cosd(Heading), sind(Heading); ...
          -sind(Heading), cosd(Heading)];
pt0lc = RotMat*[RouteMapArray(ind,1)-x_c,...
                RouteMapArray(ind,2)-y_c]';
pt1lc = RotMat*[RouteMapArray(tgt,1)-x_c,...
                RouteMapArray(tgt,2)-y_c]';
%             if pt1lc(2)<0 % in case the direction is reverse
%                 dict = -1;
%             else
%                 dict =1;
%             end
sign_stan = 0;
if sign(pt0lc(2))== sign(pt1lc(2))==1 %same side
    sign_stan = double(sign(pt1lc(2)));
elseif (sign(pt0lc(2))== sign(pt1lc(2)))==0
    sign_stan = double(sign(pt0lc(2)));
end

%e = pt1lc(2);
e = df*-sign_stan; % REASON [??]
X = zeros(4, 1);%# State  [Position_Error, Position_Error_dot, Heading_error, Heading_error_dot]
X(1, 1) =  e;   
X(2, 1) = (e - pe) / dt;
X(3, 1) =  th_e;
X(4, 1) = (th_e - pth_e) / dt;


%Curvature FeedForward  + LQR Feedback
ff = atan2(L * k, 1) ;  %[rad]
fb = -K_lqr *X;  %[rad]
%fb_a = rem((fb) ,(2*pi))-2*pi ;

P_ff = 0;
delta = P_ff*ff + fb;% delta is the Desired front wheel steering angle  [rad]
%less than 2.8 will go failure. [To be explored]

if  delta > 45/180*pi
    delta = 45/180*pi;
elseif delta < -45/180*pi
       delta = -45/180*pi;
end

return
