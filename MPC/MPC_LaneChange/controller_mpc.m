function [a ,alpha, memory] = controller_mpc(x_car,y_car,psi,v,memory,Goal_inf, N)
       %% import last step input  
        % set parameters and reference, with current state as equilibrium
Ts = 0.05;
lr = 1.7;
lf = 1.5;
     % x0 = [0 0 0 0 0 0]'     
     % last step output
a0 = memory(5); % current acceleration
theta_f0 = memory(6); %current steering angle

% update current state of the car to the memory
v0 =v;
%x0 = [x_car - memory(1), y_car - memory(2), psi - memory(3), 0, 0, 0]';
x0 = [x_car , y_car , psi , v ,0 , 0, 0, 0]'; % the last four 0 is for output
x0 = [x_car , y_car , psi , v ]'; 
        
        % Goal Position

x_goal = Goal_inf(1);
y_goal = Goal_inf(2);
theta_goal = Goal_inf(3);
v_goal =Goal_inf(4); % always want max speed. This value  should be related to the  curvature
       %% set reference according to the position of car    
      % r = [x_goal-x_car, y_goal-y_car, atan((x_goal-x_car)/(y_goal-x_car))-psi]'; % original point as the target point

       r = [x_goal, y_goal,  theta_goal, v_goal]'; % original point as the target point
       Qx = 10;
       Qy = 100;
       Qpsi = 50;
       Qv = 10;
       Rv =50; % relate to Ts
       Ralpha = 400;  % relate to Ts
       Q = diag([Qx Qy Qpsi Qv]);
       R = diag([Rv Ralpha]);
       P =diag([5 5 50 1]); % final step cost
       
       if abs(x_car - x_goal) < 5 && abs(y_car - y_goal) < 5 % apporach the destination
           Qx = 40;
           Qy = 40;
           Qpsi = 4;
           Qv =100;
           Rv =30;
           Ralpha = 300;
           Q = diag([Qx Qy Qpsi Qv]);
           R = diag([Rv Ralpha]);
          P =diag([10 10 50 40]);  % final step cost
       end
     
       
      % Bigger Q, faster reach; Bigger R, Low cost input  
%% linaerize the system
% this state space is got from the linaeriztion of the continue
% model,( Kong model  )
K = lr/(lr+lf);      
beta = atan(K*tan(theta_f0));
Ac = [0 0 -v0*sin(psi+beta) cos(psi+beta);
          0 0   v0*cos(psi+beta) sin(psi+beta);
          0 0  0 1/lr*sin(beta)
          0 0 0 0];
bedt = K/(1+K^2*(tan(theta_f0))^2)*1/((cos(theta_f0))^2); % N in notebook
Bc = [0 -v0*sin(psi+beta)*bedt;
          0 v0*cos(beta+psi)*bedt;
          0 v0/lr*cos(beta)*bedt;
         1 0];
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
C = Cd;
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
Xmax = [700  , 400, 2*pi, 7]';
Xmin = [-650, -400, -2*pi, 0.1]';
%umax = [5-v0,   0.6-alpha0]'; % maximum constrain on input
%umin = [-3-v0,    -0.6-alpha0]'; % minimum constain on input
umax = [3,   45/180*pi]'; % maximum constrain on input
umin = [-3,  - 45/180*pi]'; % minimum constain on input
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
alpha = u_mpc(2);

memory(5) = a;
memory(6) = alpha;     
%   
memory(1)=x_car;
memory(2)=y_car;
memory(3)=psi;
memory(4)=v0;
        
    end    


