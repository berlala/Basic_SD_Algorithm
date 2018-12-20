function [v ,alpha, memory] = controller_lqr(x_car,y_car,psi,memory,Goal_inf, N)
       %% import last step input  
        % set parameters and reference, with current state as equilibrium
        Ts = 0.2;
        l  = 3; %cm
     % x0 = [0 0 0 0 0 0]'     
     % last step output
        alpha0 = memory(4);
        v0 = memory(5);

        % update current state of the car to the memory

        x0 = [x_car - memory(1), y_car - memory(2), psi - memory(3), 0, 0, 0]';
        
        % Goal Position
        theta_goal = Goal_inf(3);
        x_goal = Goal_inf(1);
        y_goal = Goal_inf(2);
       %% set reference according to the position of car    
       r = [x_goal-x_car, y_goal-y_car, atan((x_goal-x_car)/(y_goal-x_car))-psi]'; % original point as the target point
       
       Qx = 5;
       Qy = 5;
       Qtheta = 100;
       Rv =2000;
       Ralpha = 100;
       Q = diag([Qx Qy Qtheta]);
       R = diag([Rv Ralpha]);
       P =diag([1000 1000 10]);
       
       if abs(x_car - x_goal) < 5&& abs(y_car - y_goal) <5
           Qx = 5;
           Qy = 5;
           Qtheta = 1000;
           Rv =100;
           Ralpha = 1000;
           Q = diag([Qx Qy Qtheta]);
           R = diag([Rv Ralpha]);
           P =diag([1000 1000 10]);  
       end
      % Bigger Q, faster reach; Bigger R, Low cost input  
%% linaerize the system
% this state space is got from the linaeriztion of the continue
% model
Ac = [0 0 -v0*sin(psi);
                  0 0 v0*cos(psi);
                   0 0 0];
Bc = [cos(psi) 0;
           sin(psi) 0;
          tan(-alpha0)/l -v0/l/(cos(alpha0))^2];
csys = ss(Ac,Bc,[],[]);
% discretize
dsys = c2d(csys,Ts);
Ad = dsys.a;
Bd = dsys.b;
Cd = eye(size(Ad,1));
        % take x(k)=[delta_x(k) y(k)];
        A = [Ad zeros(size(Ad,1));Cd*Ad eye(size(Cd))];
        B = [Bd; Cd*Bd];
        C = [zeros(size(Cd)) eye(size(Cd))];
       %% MPC model
        [Phi, Gamma] = ABN2PhiGamma(A,B,N);
        Cm =[];
        for j=1:N
            Cm = blkdiag(Cm,C);
        end
        Phi = Cm*Phi; 
        Gamma = Cm*Gamma;
        [ Psi, Omega ] = QRPN2PsiOmega(Q,R,P,N);%use Q instead of P
        %% generate reference matrix according to predict horizon
Rref = [];
for i=1:N
       Rref = [Rref;r];
end

G = 2*(Psi+Gamma'*Omega*Gamma);
F = 2*Gamma'*Omega*(Phi*x0-Rref);
%% apply constraints
Xmax = [700  , 400, 2*pi-psi]';
Xmin = [-650, -400, -2*pi-psi]';
umax = [20-v0,   0.6-alpha0]';
umin = [1-v0,    -0.6-alpha0]';
[ W, L, c] = getWLcR(C,Bd, Xmax, Xmin, umax, umin, Gamma, Phi);
%% solve the constraint mpc
b = c+W*x0;% L*U<=b=c+W*x0
U = quadprog(G,F',L,b);
delta_u = [eye(2) zeros(2,2*N-2)]*U;
u_mpc = delta_u+[v0;alpha0]
        
v = u_mpc(1) ;
alpha = u_mpc(2);
        
memory(4) = alpha;     
memory(5) = v;

%   
memory(1)=x_car;
memory(2)=y_car;
memory(3)=psi;
        
    end    


