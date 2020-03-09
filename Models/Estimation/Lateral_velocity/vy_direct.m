%% Direct Obsever
%%
Cf = 15000;
Cr = 15000;
m = 1700;
vx = 60/3.6;
lf = 1.4;
lr = 1.5;

C  = [-(Cf+Cr)/(m*vx), (-lf*Cf+lr*Cr)/(m*vx);
        0 ,1];
D = [Cf/m;0];

ay = 0.5; % lateral acceletaion
r = 0.2; % yaw-rate
theta = 0.1; % the steering wheel [rad]
y_mes = [ay;r];

x_obs = inv(C)*[y_mes - D*theta]; 

