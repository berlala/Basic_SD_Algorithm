%% Simulation set-up for the 
clear all;close all;clc
warning off
%% initilize the system
Ts = 0.1;
N =10;
%Initial Position and Heading
x_car =-5;
y_car =-5;
psi =0;         
v= 1.0;

a_ini = 0.0; % initial acceleration
alpha_ini= 0; % last steering angle 
memory = [x_car,y_car,psi, v, a_ini, alpha_ini]; % Last state
%% The destination
x_goal = 30; 
y_goal = 30;
psi_goal =pi; 
v_goal = 0.1; % stop at the end 

Goal_inf = [x_goal,y_goal,psi_goal, v_goal]; 

figure(1)
plot(x_car,y_car,'r*',x_goal,y_goal,'ro')
grid on; 
%legend('Initial Position','Goal','Location','best');
hold on;
%% MPC control iterations
x_h = [];
y_h = [];
v_h = [];
delta_h = [];
a_h = [];
h =2;
i = 1;
while i   % Control iteration
    
    %updtae the initial state
    x_car0 = x_car;
    y_car0 = y_car;
    psi0 = psi;
    v_car0 = v;
    
% [v ,alpha, memory] = controller_lqr(x_car,y_car,psi,memory,Goal_inf, N);
  [a ,alpha, memory] = controller_lqr2(x_car,y_car,psi,v ,memory,Goal_inf, N);

   a_cmd = a
   delta_f = alpha
    
    sim bic_3.slx
    
    x_car=x_state.Data(h);
    y_car=y_state.Data(h);
    % plot(x_state.Data, y_state.Data)
    psi= psi_state.Data(h);
    v = v_state.Data(h);
    %plot(v_state.Data);

   
    x_h = [x_h;x_car];
    y_h = [y_h;y_car];
    v_h = [v_h;v];
    delta_h = [delta_h, delta_f];
    a_h = [a_h, a];
    
    i = i+1;
    if (abs(x_car - x_goal) <1&& abs(y_car - y_goal) <1 )&&abs(psi - psi_goal) <0.1 ||i >200
        disp('reach goal')
        break
    end
 
 figure(1)
 plot(x_car,y_car, 'ob');hold on
    
end

figure(1)
plot(x_h, y_h,'ob-');
figure
subplot(2,1,1)
plot(v_h);
ylabel('velocity')
subplot(2,1,2)
plot(delta_h);
ylabel('Psi')

