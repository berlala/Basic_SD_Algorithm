% Mu Estimate
%%
function [Fyf, Fyr] = lateral_force_fcn(ay, yaw_rate_dot)

ll = lf+lr;
Fyf = [];
Fyr = [];

% Parameters 
sw_ratio = 15.4;
lf = 1.328;
lr = 1.482;
mf = 580*2;
mr = 580*2;
m = mr+mf;
Iz = ((lf+lr)*(mf/m))^2*mf + (ll*(mr/m))^2*mr;

for i = 1:length(ay)
 theta  = steering(i)/sw_ratio/180*3.14;
Fyf(i) = (m*lr/ll * ay(i) + Iz/ll * yaw_rate_dot(i))/cos(theta);
Fyr(i) = m*lf/ll*ay(i)  - Iz/ll*yaw_rate_dot(i);
end


end