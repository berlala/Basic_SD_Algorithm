%% bicycle model
%%
function [x_1, y_1, phi_1, v_1] = bicycle_model(x, y, phi,v, theta_f, Ts)

lr = 1.5028;
ll = 1.5028+1.3472;
%dt = 0.002; % [s]
dt = Ts;

beta = atan(lr/(ll) * tan(theta_f));
x_1 = x + v*cos(phi+beta)*dt;
y_1 = y+v*sin(phi+beta)*dt;
phi_1 = phi+v/lr*sin(beta)*dt;
v_1 = v;


end