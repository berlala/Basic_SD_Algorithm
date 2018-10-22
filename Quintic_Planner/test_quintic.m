% Test Quintic Planner
%%
% Bolin Zhao, ff china, bolin.zhao@ff.com
%%
clc;clear;
%%
T =  5;
t = 0.1;

% Start State
xs = 0;
vxs = 0;
axs = 0.1;
ys = 0;
vys = 1;
ays = 0;


% End State
xg = 50;
vxg = 1;
axg =0;
yg = 10;
vyg = 1;
ayg = 0;

% Path Geneartion

[xt] = quintic_planner(xs, vxs,axs,xg,vxg,axg,T,t) ;
[yt] = quintic_planner(ys, vys,ays,yg,vyg,ayg,T,t) ;

%% Result
plot(xt,yt)
title('')