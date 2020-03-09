function [v alpha, memory, plotting] = controller(x_car,y_car,theta,x_ball,y_ball,memory)
    plotting = []; % Makes sure this output exists
    if x_car>70 || x_car<-65
        v=0;
        alpha=0;
        disp('out of border')
    elseif y_car>40 ||y_car<-40
        v=0;
        alpha=0;
        disp('out of border')
    else % run mpc if car is within the area
       %% import last step input  
        % set parameters and reference, with current state as equilibrium
        Ts = 0.2;
        l = 11.1;
        if isempty(memory) % for initialization when run the real car
            x0 = [0 0 0 0 0 0]';  % initial state
            memory = [0 0 0 1 0 0 0 0];
        else 
            x0 = [x_car-memory(1) y_car-memory(2) theta-memory(3) 0 0 0]';
        end
        v0 = memory(4); % last step output
        alpha0 = memory(5);
        % update current state of the car to the memory
        memory(1)=x_car;
        memory(2)=y_car;
        memory(3)=theta;
       %% set reference according to the position of ball and car
        x_goal = 70; 
        y_goal = 0;
        xgr = x_goal-x_ball
        ygr = y_goal-y_ball
        theta_goal = atan(ygr/xgr) % calculate theta for goal
        l_pre = 20; % set a pre reference point for car
        
        if memory(8)==0
            y_p = y_ball-l_pre*sin(theta_goal);
            x_relax = 0.25*abs(y_car-y_p);
            x_p = x_ball-l_pre*cos(theta_goal)-x_relax;
            if x_car > (x_p-30)
                r = [-60-x_car y_p-y_car 0-theta]';% state ref
            
                Q = diag([4 5 1000]);
                R = diag([1000 100]);
                P = diag([1 1 1000]);
            else
                memory(6) = x_p;
                memory(7) = y_p;
                memory(8) = 1;
                xpr = x_p-x_car;
            ypr = y_p-y_car;
                r = [xpr ypr (atan(ygr/xgr)-theta)]';% state ref
                Q = diag([4 5 1000]);
                R = diag([1000 100]);
                P = diag([1 1 1000]);
            end
        else
            x_p = memory(6);
            y_p = memory(7);
            xr = x_ball-x_car;
            yr = y_ball-y_car;
            xpr = x_p-x_car;
            ypr = y_p-y_car;

            if x_car<x_p % car far away
                r = [xpr ypr (atan(ygr/xgr)-theta)]'% state ref
                
                Qx = 4;
                Qy = 5;
                Qtheta = 100;
                Rv =3000;
                Ralpha = 100;
                Q = diag([Qx Qy Qtheta]);
                R = diag([Rv Ralpha]);
                P = diag([1 1 100]);
            else % car reach pre ref point
                r = [xr yr (atan(ygr/xgr)-theta)]'% state ref
                Qx = 5;
                Qy = 5;
                Qtheta = 100;
                Rv =2000;
                Ralpha = 10;
                Q = diag([Qx Qy Qtheta]);
                R = diag([Rv Ralpha]);
                P = Q;%diag([1 1 1000])
            end
        end
            
      
        
        

       %% linaerize the system
        Ac = [0 0 -v0*sin(theta);0 0 v0*cos(theta);0 0 0];
        Bc = [cos(theta) 0;sin(theta) 0; tan(-alpha0)/l -v0/l/(cos(alpha0))^2];
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
        N = 10;
        [ Phi, Gamma ] = ABN2PhiGamma(A,B,N);
        Cm =[];
        for j=1:N
            Cm = blkdiag(Cm,C);
        end
        Phi = Cm*Phi; 
        Gamma = Cm*Gamma;
        [ Psi, Omega ] = QRPN2PsiOmega(Q,R,P,N);%use Q instead of P

        % generate reference matrix according to predict horizon
        Rref = [];
        for i=1:N
            Rref = [Rref;r];
        end

        G = 2*(Psi+Gamma'*Omega*Gamma);
        F = 2*Gamma'*Omega*(Phi*x0-Rref);

       %% apply constraints
        Xmax = [70-x_car 40-y_car 2*pi-theta]';
        Xmin = [-65-x_car -40-y_car -2*pi-theta]';
        umax = [50-v0 0.6-alpha0]';
        umin = [-10-v0 -0.6-alpha0]';
        [ W, L, c] = getWLcR( C,Bd, Xmax, Xmin, umax, umin, Gamma, Phi);

       %% solve the constraint mpc
        b = c+W*x0; % L*U<=b=c+W*x0
        U = quadprog(G,F',L,b);
        delta_u = [eye(2) zeros(2,2*N-2)]*U;
        u_mpc = delta_u+[v0;alpha0];
        v = u_mpc(1);
        alpha = u_mpc(2);
        memory(4) = v;
        memory(5) = alpha;
        
       
%     % Additional plots can be called in the following way 
%     % (uncomment, or remove if you want to)
%     plotting{1}.x = [-5 5];
%     plotting{1}.y = [-5 5];
%     plotting{1}.settings = 'b--';
%     
%     plotting{2}.x = [-5 5];
%     plotting{2}.y = [5 -5];
%     plotting{2}.settings = 'r';
    end    
end

