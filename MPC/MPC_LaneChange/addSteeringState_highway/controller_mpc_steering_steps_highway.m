function [a ,s, memory] = controller_mpc_steering_steps_highway(x_car,y_car,psi,v,alpha,memory,Goal_inf, N)
       %% import last step input  
        % set parameters and reference, with current state as equilibrium
Ts = 0.05;
lf = 1.5;
lr = 1.7;
spd_limit = 120; %km/h, for highway

% x0 = [0 0 0 0 0 0]'     
% last step output
a0 = memory(6); % current acceleration
s0 = memory(7); %current steering angle changing rate

% update current state of the car to the memory
v0 =v;
alpha0 = alpha;
%x0 = [x_car - memory(1), y_car - memory(2), psi - memory(3), 0, 0, 0]';
%x0 = [x_car , y_car , psi , v ,0 , 0, 0, 0]'; % the last four 0 is for output
x0 = [x_car , y_car , psi , v, alpha ]'; 
        
% Goal Position

x_goal = Goal_inf(1);
y_goal = Goal_inf(2);
theta_goal = Goal_inf(3);
v_goal =Goal_inf(4); % always want max speed. This value  should be related to the  curvature
alpha_goal = Goal_inf(5);
%% set reference according to the position of car    
% r = [x_goal-x_car, y_goal-y_car, atan((x_goal-x_car)/(y_goal-x_car))-psi]'; % original point as the target point
r = [x_goal, y_goal,  theta_goal, v_goal, alpha_goal]'; % original point as the target point

Qx = 0.2;
Qy = 0.1;
Qpsi = 1;
Qv = 1;
Qalpha =0;
Rv = 0.01; % relate to Ts
Rs = 0.01  % relate to Ts
Q = diag([Qx Qy Qpsi Qv Qalpha]);
R = diag([Rv Rs]);
P =diag([1 1 1 1 1]); % final step cost
       
if abs(x_car - x_goal) < 5 && abs(y_car - y_goal) < 0.2 % apporach the destination
    Qx = 4;
    Qy = 4;
    Qpsi = 8; %heading
    Qv =10;
    Qalpha = 10; %steering angle
    Rv =3;
    Rs = 6; % steering angle rate
    Q = diag([Qx Qy Qpsi Qv Qalpha]);
    R = diag([Rv Rs]);
    P =diag([1 1 5 2 2]);  % final step cost
end

      % Bigger Q, faster reach; Bigger R, Low cost input  
%% linaerize the system
% this state space is got from the linaeriztion of the continue
% model,( Kong model  )
K = lr/(lr+lf);      
beta = atan(K*tan(alpha0));
bedt = K/(1+K^2*(tan(alpha0))^2)*1/((cos(alpha0))^2); % N in notebook
Ac = [0, 0, -v0*sin(psi+beta), cos(psi+beta), -v0*sin(psi+beta)*bedt;
         0, 0,  v0*cos(psi+beta), sin(psi+beta),  v0*cos(psi+beta)*bedt;
         0, 0,                          0, 1/lr*sin(beta),     v0*cos(beta)/lr*bedt;
         0, 0,                          0,                   0,                                   0;
         0, 0,                          0,                   0,                                   0];

Bc = [0, 0;
         0, 0;
         0, 0;
         1, 0;
         0,1];
csys = ss(Ac,Bc,[],[]);
% discretize
dsys = c2d(csys,Ts);
Ad = dsys.a;
Bd = dsys.b;
%Cd = eye(size(Ad,1));
Cd = zeros(size(Ad,1));
        % take x(k)=[delta_x(k) y(k)];
%A = [Ad zeros(size(Ad,1));Cd*Ad eye(size(Cd))];
%B = [Bd; Cd*Bd];
%C = [zeros(size(Cd)) eye(size(Cd))];
A = Ad;
B = Bd;
%C = Cd;
%C = [zeros(size(Cd)) eye(size(Cd))];
%% MPC model
[Phi, Gamma] = ABN2PhiGamma(A,B,N);
%Cm =[];
%for j=1:N
%   Cm = blkdiag(Cm,C);
% end
%Phi = Cm*Phi; 
%Gamma = Cm*Gamma;
[ Psi, Omega ] = QRPN2PsiOmega(Q,R,P,N);%use Q instead of P
 %% generate reference matrix according to predict horizon
Rref = [];
for i=1:N
       Rref = [Rref;r];
end
G = 2*(Psi+Gamma'*Omega*Gamma);
F = 2*Gamma'*Omega*(Phi*x0-Rref);
%% apply constraints
%Xmax = [700  , 400, 2*pi-psi]';
%Xmin = [-650, -400, -2*pi-psi]';
Xmax = [105, 5.625 , 2*pi, spd_limit/3.6, 45/180*pi]'; % state constrain, X, Y, heading, velocity, Front Wheel Angle
Xmin = [0,-1.875,  -2*pi, 0.1,-45/180*pi]';
%umax = [5-v0,   0.6-alpha0]'; % maximum constrain on input
%umin = [-3-v0,    -0.6-alpha0]'; % minimum constain on input
if psi >5/180*pi
    umax = [4*Ts,   1.6*180/180*pi*Ts]'; % maximum constrain on input, should take time simple into account
    umin = [-8*Ts,  1.6*-180/180*pi*Ts]'; % minimum constain on input
else
    umax = [4*Ts,   0.8*180/180*pi*Ts]'; % maximum constrain on input, should take time simple into account
    umin = [-8*Ts,  0.8*-180/180*pi*Ts]'; % minimum constain on input
end

[ W, L, c] = getWLcR(Bd, Xmax, Xmin, umax, umin, Gamma, Phi);
%% solve the constraint mpc
b = c+W*x0;% L*U<=b=c+W*x0
U = quadprog(G,F',L,b);
if isempty(U)
    disp ('No go!' )
end
%%
delta_u = [eye(2) zeros(2,2*N-2)]*U;
u_mpc = delta_u;%+[v0;alpha0]
        
a= u_mpc(1) ;
s = u_mpc(2);

memory(6) = a;
memory(7) = s;     
%   
memory(1)=x_car;
memory(2)=y_car;
memory(3)=psi;
memory(4)=v0;
memory(5)=alpha0;
        
end    


