%% Simulation set-up for the 
clear all;close all;clc
warning off
% 2019.5.14: add smooth function for steering control

%% initilize the system
Ts = 0.05;
N =20;
%Initial Position and Heading
x_car =0;
y_car =0;
psi =0;         
v= 5.0; % m/s

a_ini = 1.0; % initial acceleration
alpha_ini= 0; % last steering angle 
memory = [x_car,y_car,psi, v, a_ini, alpha_ini]; % Last state
%% The destination
x_goal = 50; 
y_goal = 3.75;
psi_goal =0; 
v_goal = 5.0; % stop at the end 

Goal_inf = [x_goal,y_goal,psi_goal, v_goal]; 
%% Initial Plot
line1 = [0:0.1:55;ones(1,551)*-1.875];
line2 = [0:0.1:55;ones(1,551)*1.875];
line3 = [0:0.1:55;ones(1,551)*5.625];
center1 =  [0:0.1:55;zeros(1,551)];
center2 =  [0:0.1:55;ones(1,551)*3.75];


figure(1)
plot(x_car,y_car,'r*',x_goal,y_goal,'ro')
grid on; 
%legend('Initial Position','Goal','Location','best');
hold on;
plot(line1(1,:), line1(2,:),'b','linewidth',2)
plot(line2(1,:), line2(2,:),'c','linewidth',2)
plot(line3(1,:), line3(2,:),'b','linewidth',2)
plot(center1(1,:), center1(2,:),'g--','linewidth',2)
plot(center2(1,:), center2(2,:),'g--','linewidth',2)
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
 % [a ,alpha, memory] = controller_mpc(x_car,y_car,psi,v ,memory,Goal_inf, N);
 [a ,alpha, memory] = controller_mpc_Tunner(x_car,y_car,psi,v ,memory,Goal_inf, N);

   a_cmd = a
   %delta_f = alpha
   gain = 1.00;
   delta_f  = gain*smooth_steering([delta_h,alpha])
    
    sim bic_3.slx
    % can be replaced by a discrete vehicle model
    
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
    if (abs(x_car - x_goal) <0.2&& abs(y_car - y_goal) <0.2)&&abs(psi - psi_goal) <0.1 ||i >10000
        disp('reach goal')
        break
    end
 
 figure(1)
 plot(x_car,y_car, 'ob');hold on
    
end
%% final plot

time = Ts*(0:1:length(v_h)-1);

figure(1)
plot(x_h, y_h,'ob-');
xlabel('X[m]')
ylabel('Y[m]')
title('Path')
figure
subplot(2,1,1)
plot(time,v_h*3.6);
ylabel('Velocity[km/h]')
subplot(2,1,2)
plot(time,delta_h(1:length(time))/pi*180);
ylabel('Front Wheel Steering Angle[Deg]')

