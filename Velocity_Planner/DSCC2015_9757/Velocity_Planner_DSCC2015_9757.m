%% Velocity Planner DSCC2015-9757
%% 3 step velocity planner for racing purpose
%% 
close all;clear;clc
load track_bl.mat;

% Line_x  and Line_y are the coordinate of the track.
% Line_K is the curvature of the track.

%% Clear Data
% To remove the possible same coordinate in the origin data
for i = 1: (length(Line_x)-1)
    if (Line_x(i) == Line_x(i+1))&&(Line_y(i) == Line_y(i+1))
        Line_x(i) = (Line_x(i-1)+Line_x(i+1))/2;
        Line_y(i) = (Line_y(i-1)+Line_y(i+1))/2;
        Line_K(i) = (Line_K(i-1)+Line_K(i+1))/2;
    end
end

if (Line_K(end) ~= Line_K(1)) 
    Line_K(end) = min(Line_K(1), Line_K(end));
    Line_K(1) = Line_K(end);  
end

figure(2)
subplot(2,1,1)
plot(Line_x, Line_y)
subplot(2,1,2)
plot(Line_K)
%% Perpare step
s = [];s_aum = [];
s(1) = 0;
s_aum(1) = 0;
for i = 2:length(Line_K)
    s_step = sqrt((Line_x(i)-Line_x(i-1))^2 + (Line_y(i)-Line_y(i-1))^2 );
    s(i) = s_step;
    s_aum(i)  = s_aum(end)+s_step;
end

%% Step1
 u = 0.8; %normal road surface
 U = [];
 g = 9.8;%m/s^2
for i  = 1:length(s_aum)
    if abs(Line_K(i)) < 0.002
        K = 0.002;
    else
        K = abs(Line_K(i));
    end
    
    U(i) = sqrt(u*g/K);
end
figure(1)
plot(s_aum, U); hold on
xlabel('Distance[m]')
ylabel('Velocity[m/s]')
%% Step2: Forward Check Acceleration Ability

for i = 2:length(U)
    U_forward= sqrt(U(i-1)^2 + 2*fcn_acc(U(i-1))*s(i-1));
    U(i) = min(U(i), U_forward);
end
plot(s_aum, U); hold on

%% Step3: Forward Check Deceleration Ability
for i = length(U)-1:-1:1
    U_backward= sqrt(U(i+1)^2 + 2*fcn_del(U(i+1))*s(i+1));
    U(i) = min(U(i), U_backward);
end
plot(s_aum, U,'linewidth',2); hold on
legend('Inital', 'Forward','Backward(Final)')
