%% Pure Pursuit Algorithm 
% Bolin 
clear;close all
%% Path which need to follow
%T =0.1;
%t = 0:T:200;
%x = 1*t;
%y = 5*ones(1,length(x));
% plot(x,y);
% hold on
%% The Pure Pursuit Method
P_ini = [3,0];
% plot(P_ini(1), P_ini(2), 'o')
velocity = 0.5; 
l=2.5; 
L = 10;

sim('bicycle')
%% Analysis the Result
plot(X_ref, Y_ref,'*');
hold on
plot(X_p, Y_p,'o-')
