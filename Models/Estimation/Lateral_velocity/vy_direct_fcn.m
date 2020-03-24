%% Direct Obsever
%%
function [x_obs] = vy_direct_fcn(vx, y_mes, theta)
% This function is only valid for the linear  region

%MKZ Read Data 
% Cf = 1.2436e5;
% Cr = 1.4889e5;
% lf = 2.4;
% lr = 0.1;
% m = 2550;

%MKZ CarSim data
Cf = 2*4.8e4; %Axle Cf
Cr =  2*5.5e4;
lf = 1.328;
lr = 1.482;
mfl = 5700/9.8;
mfr = mfl;
mf = mfl+mfl;
mrl = 5110/9.8;
mrr = mrl;
mr = mrl+mrr;
m = mr+mf;


C  = [-(Cf+Cr)/(m*vx), (-lf*Cf+lr*Cr)/(m*vx);
        0 ,1];
D = [Cf/m;0];

x_obs = inv(C)*[y_mes - D*theta]; 

end