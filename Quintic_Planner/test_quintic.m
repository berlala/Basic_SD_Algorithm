% Test Quintic Planner
%%
% Bolin Zhao, ff china, bolin.zhao@ff.com
%% Config
clc;clear;close all
V_Threshold =30;
Acc_Threshold =3; % m/s^2
Jerk_Threshold = 0.5; % m/s^3
T_Max =  100; % Max Allow Time 
t = 0.1;
%% States
% Start State
xs = 10;
ys = 10;
vxs =-1;
vys = 0;
axs =0.1;
ays = 0;


% End State
xg = 50;
yg = 20;
vxg = 10;
vyg = 0;
axg =0;
ayg = 0;

%% Path Geneartion

for T =1:1:T_Max
Result_true = 0;
[xt,vxt,axt,jxt] = quintic_planner(xs, vxs,axs,xg,vxg,axg,T,t) ;
[yt,vyt,ayt,jyt] = quintic_planner(ys, vys,ays,yg,vyg,ayg,T,t) ;

for i = 1:length(xt)
    if sqrt(axt(i)^2+ayt(i)^2)>Acc_Threshold|| sqrt(jxt(i)^2+jxt(i)^2)>Jerk_Threshold || sqrt(vxt(i)^2+vyt(i)^2)>V_Threshold
        Result_true = 1 ; % the condition to accept the effective result.
    else
        Result_true = Result_true;
    end
end
if Result_true ==0
    break
end

end
%% Result
plot(xt,yt);hold on
title('Local Path Map')
quiver(xs,ys,vxs,vys,'linewidth',2)
quiver(xg,yg,vxg,vyg,'linewidth',2)
xlabel('X')
ylabel('Y')

v = [sqrt(vxs^2+vys^2)];
a = [sqrt(axs^2+ays^2)];
jerk =[0];
for ii = 2:length(vxt)
    v(ii) = sqrt(vxt(ii)^2+vyt(ii)^2);
    if v(ii) - v(ii-1)<0 % deceleration
        dec = -1;
    else
        dec = 1;
    end
    a(ii) = sqrt(axt(ii)^2+ayt(ii)^2)*dec;
    if a(ii) - a(ii-1)<0 % deceleration
        dec_a = -1;
    else
        dec_a = 1;
    end
    jerk(ii) = sqrt(jxt(ii)^2+jxt(ii)^2)*dec_a;
end

time = (0:1:length(v)-1)*t;
figure(2)
plot(time, v);hold on
plot(time,a)
plot(time,jerk)
legend('Velocity[m/s]','Acceleration','Jerk')