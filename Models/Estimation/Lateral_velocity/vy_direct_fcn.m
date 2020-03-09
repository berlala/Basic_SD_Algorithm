%% Direct Obsever
%%
function [x_obs] = vy_direct_fcn(vx, y_mes, theta)

%MKZ Read Data 
% Cf = 1.2436e5;
% Cr = 1.4889e5;
% lf = 2.4;
% lr = 0.1;
% m = 2550;

%MKZ CarSim data
Cf = 1e5;
Cr = 1.1e5;
lf = 1.328;
lr = 1.482;
m = 580*4;


C  = [-(Cf+Cr)/(m*vx), (-lf*Cf+lr*Cr)/(m*vx);
        0 ,1];
D = [Cf/m;0];

x_obs = inv(C)*[y_mes - D*theta]; 

end