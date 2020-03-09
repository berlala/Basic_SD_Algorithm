function [delta,e, th_e,ind,tgt,CAL] = MPC_bl(initial_ENH,v, RouteMapArray, CurvDerVehMap, pe, pth_e,Preview,delta_f) 

yalmip('clear')
% Ref: https://yalmip.github.io/example/standardmpc
CAL = {};
%%  Model data
% Default Paramters
dt = 0.1; % sample time
lr = 1.5;
lf = 1.7;
Iz = 3600;
Cf = 36600;
Cr = Cf;
m  = 2300;
L = lr+lf;% wheelbase

x_c = initial_ENH(1);
y_c = initial_ENH(2);
psi_c = initial_ENH(3);

%% Config

%Preview  =1; % [horizonX, to be updated]
Heading = (psi_c )*180/pi;
RotMat = [cosd(Heading), sind(Heading); ...
          -sind(Heading), cosd(Heading)];
[ind,~,X_clst] = find_closest2(RouteMapArray,initial_ENH);      
tgt = min(ind+Preview, length(RouteMapArray(:,1)));

pt0lc = RotMat*[X_clst(1)-x_c,...
                X_clst(2)-y_c]';
pt1lc = RotMat*[RouteMapArray(tgt,1)-x_c,...
                RouteMapArray(tgt,2)-y_c]';
            
th_e =( initial_ENH(3)- RouteMapArray(ind,3)); % Heading error [rad]
e = (0-pt1lc(2)); % Lateral Error

%e = df*-sign_stan; % Without [-sign_stan], it may cannot strict follow
Xs = zeros(4, 1);%# State  [Position_Error, Position_Error_dot, Heading_error, Heading_error_dot]
Xs(1, 1) =  e;   
Xs(2, 1) = (e - pe) / dt;
Xs(3, 1) =  th_e;
Xs(4, 1) = (th_e - pth_e) / dt;

%target information[Global]
x_tg = RouteMapArray(tgt,1);
y_tg = RouteMapArray(tgt,2);
psi_tg= RouteMapArray(tgt,3);
k = CurvDerVehMap(tgt);

Theta_dot = v*k;

%%
% This SS model should be discrete
Ac = [0, 1, 0, 0;
         0, -(2*Cf+2*Cr)/(m*v), (2*Cf+2*Cr)/m, (-2*Cf*lf + 2*Cr*lr)/(m*v);
         0, 0, 0 ,1;
         0, -(2*Cf*lf-2*Cr*lr)/(Iz*v), (2*Cf*lf-2*Cr*lr)/Iz, -(2*Cf*lf^2+2*Cr*lr^2)/(Iz*v)];
B = sdpvar(4,1);
E = [0;0;0;0];
%K = [0, -(2*Cf*lf-2*Cr*lf)/(m*v)-v, 0, -(2*Cf*lf^2+2*Cr*lr^2)/(Iz*v)]';

nx = 4; % Number of states
nu = 1; % Number of inputs

 Bc = [0, 2*Cf/m, 0, 2*Cf*lf/Iz]';
 sys = ss(Ac,Bc,[] ,[]);
 sysd = c2d(sys,dt);
 Ad = sysd.a;
 Bd = sysd.b;
 %Kd = syss.d;

% MPC data
Q = eye(4);
Q(1,1) =1;
Q(2,2) =0;
Q(3,3) =0.1;
Q(4,4) =0;
R = 20; % increase R to reduce the flucation of the cmd
N =10; % horizon

ny = 4; %observer
C = [1 1 1 1]; % only two states can be got

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
r = sdpvar(repmat(ny,1,N+1),repmat(1,1,N+1));
d = sdpvar(1);
pastu = sdpvar(1);

cons=100/180*pi*dt;
constraints = [-cons<= diff([pastu u{:}]) <= cons]; % steering wheel changing rate/sample
objective = 0;

for k = 1:N
    objective = objective + (C*x{k}-r{k})'*Q*(C*x{k}-r{k}) + u{k}'*R*u{k}; % Objective is cost function, the error between the Model predctive and reference error
    constraints = [constraints, x{k+1} == Ad*x{k}+Bd*u{k}+E*d]; % A discrete model
    constraints = [constraints, -45/180*pi<= u{k}<= 45/180*pi, [-5,-10, -pi/3, -pi]'<=x{k+1}<=[5,10, pi/3, pi]'];
end
objective = objective + (C*x{N+1}-r{N+1})'*Q*(C*x{N+1}-r{N+1}); % final cost

parameters_in = {x{1},[r{:}],d,pastu,B}; % input definition
solutions_out = {[u{:}], [x{:}]}; % output definition

controller = optimizer(constraints, objective,sdpsettings('solver','ipopt'),parameters_in,solutions_out);
disturbance = 0.000*randn(1);
oldu = delta_f;

%Bmodel = [0, 2*Cf/m, 0, 2*Cf*lf/Iz]';
Bmodel = Bd;
Reference = zeros(4,N+1);
inputs = {Xs,Reference,disturbance,oldu,Bmodel};
[solutions,diagnostics] = controller{inputs};
U = solutions{1};
X = solutions{2};
if diagnostics == 1
    disp('The problem is infeasible'); % do not use error
else
    disp('Solved')
end
disturbance = 0.0*disturbance + 0.000*randn(1)*0.1;
    
 delta = U(1);
 
 CAL{1} = U;
 CAL{2} = X;
end