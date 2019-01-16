%% For Path And Velocity Generation
% Bolin, 2019.1.14
% In thie version, the velocity is planned by a Quintic Planner.
% this tool is used  to generate the desire path [RouteMapArray] for
% simulation
clc;
clear;close all
load_spiral_init_para;
%%  Config
% Data: [X, Y, Heading, Curvature(steeringwheel), Velocity ]
% In Path_points, pls. edit the key point that the path need to be passed.
Path_points = [0,0,0,0,3/3.6;
                           10,0,0,0,2/3.6];
%%
step_length = 0.2;
v_max = 10/3.6;
RouteMapArray_gen = [];

t = 0.01;

%%
for i = 1:length(Path_points(:,1))-1
    
start_point =Path_points(i,:); % Initial Global [X, Y, Heading, Curvature, Speed]
end_point = Path_points(i+1,:); % Final Gloabl [X, Y, Heading, Curvature, Speed]

[q, Spiral_Msg ] = fuc_spiral_generation(start_point, end_point, step_length,threeD_spiral_param);

%v_profile = fuc_velocity_planing(Spiral_Msg(:,4:5),start_point(5), end_point(5), v_max, step_length);
axs = 0;
axe = 0;
T_max = length(Spiral_Msg(:,1));

for T = 1:0.5:T_max % change in this loop
[~, ~, at_log, jt_log] = quintic_planner_x(0, start_point(5),axs,Spiral_Msg(end,5),end_point(5),axe,T,t);
if max(at_log)<0.5&&max(jt_log)<1
    break
end
end

t_final = T/T_max;
[xt_log, vt_log, at_log, jt_log] = quintic_planner_x(0, start_point(5),axs,Spiral_Msg(end,5),end_point(5),axe,T,t_final);
v_final = vt_log(1:T_max)';
a_final = at_log(1:T_max)';
T_final  = t_final*(0:1:T_max-1);

path = [Spiral_Msg,v_final,a_final]; % x y theta k s v a
% figure(1);
% plot(path(:,1), path(:,2));

% title('Generated Path')
% xlabel('X[m]')
% ylabel('Y[m]')
% figure(2);
% [hAy,hLine1,hLine2] = plotyy(path(:,5),path(:,6),path(:,5),path(:,7));
% xlabel('distance[m]')
% ylabel(hAy(1),'Velocity[m/s]') 
% ylabel(hAy(2),'Acceleration[m/s^2]') 
% title('Velocity and Acceleration')

% figure(2)
instance_s = [0;diff(path(:,5))];
clear t_v
t_v(1) = 0;
for ii = 2:length(path(:,6))-1
t_ins = instance_s(ii+1)/path(ii,6); 
t_v(ii) = t_v(end)+t_ins;
end

%% The final RouteMapArray to Control Level (Data Format)
RouteMapArray = double(path);
Local_XY = RouteMapArray(:,1:2);

Heading = (RouteMapArray(1,3)-start_point(3) )*180/pi;
RotMat = [cosd(Heading), sind(Heading); ...
                   -sind(Heading), cosd(Heading)];
Local_GB = [];
for i = 1:length(Local_XY)
    Local_GB(i,:) =   RotMat*[Local_XY(i,1), ...
                                                   Local_XY(i,2)]';
    Local_GB(i,:)  =  Local_GB(i,:) +    start_point(1:2);        
end
Heading_GB = RouteMapArray(:,3)+start_point(3);
if ~isempty(RouteMapArray_gen) 
RouteMapArray(:,5)  = RouteMapArray(:,5) + RouteMapArray_gen(end,5);
end
RouteMapArray_m   = [Local_GB,Heading_GB,RouteMapArray(:,4:end)] ;  
RouteMapArray_gen = [RouteMapArray_gen;RouteMapArray_m];
clear  RouteMapArray_m
end % end  of the iteration of generation 

%% Plot
figure(4)
title('Sprial Path + Quintic Velocity')
subplot(4,1,1:2);
plot(RouteMapArray_gen(:,1),RouteMapArray_gen(:,2))
title('Path')
xlabel('EAST')
ylabel('NORTH')
subplot(4,1,3)
plot(RouteMapArray_gen(:,3)/pi*180)
ylabel('Heading[deg]')
subplot(4,1,4)
plot(RouteMapArray_gen(:,5), RouteMapArray_gen(:,6))
xlabel('Distance[m]')
ylabel('Velocity[m/s]')
title('Velocity[m/s]')

figure(2)
subplot(2,1,1)
plot(T_final, RouteMapArray_gen(:,6));
xlabel('Time[s]')
subplot(2,1,2)
plot(T_final, RouteMapArray_gen(:,7));
