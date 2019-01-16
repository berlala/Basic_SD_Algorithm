% Test Quintic Planner
%%
% Bolin Zhao, ff china, bolin.zhao@ff.com
%% Config
clc;clear;close all
V_Threshold =30;
Acc_Threshold =3; % m/s^2
Jerk_Threshold = 0.5; % m/s^3
Str_Threshold = 0.2;
T_Max =  200; % Max Allow Time 
t = 0.01;
%% States
% Start State
xs = 0;
ys = 0;
vxs = 2;
vys = 0;
axs =0.1;
ays = 0;


% End State
xg = 0;
yg =  30;
vxg =-2;
vyg =0;
axg =0;
ayg =0;

%% Path Geneartion

for T =1:1:T_Max
Result_true = 0;
[xt,vxt,axt,jxt] = quintic_planner(xs, vxs,axs,xg,vxg,axg,T,t) ;
[yt,vyt,ayt,jyt] = quintic_planner(ys, vys,ays,yg,vyg,ayg,T,t) ;

K2 = [];
for i = 1:length(xt)

 % Calculate the Curvature 
   if i-1<1
      i =2;
  else if i+1> length(xt)
          i =  length(xt)-1;
      end
  end
  x_0 = xt(i-1);
  x_1 = xt(i);
  x_2 = xt(i+1);
  y_0 = yt(i-1);
  y_1 = yt(i);
  y_2 = yt(i+1);

 % Method (1)
Heading_t = atan((y_2-y_0)/(x_2-x_0));
RotMat = [cos(Heading_t), sin(Heading_t); ...
                 -sin(Heading_t), cos(Heading_t)];
A_rot = RotMat*[x_1-x_0,y_1-y_0]';
if A_rot(2) >0
    d  = -1;
else
    d = 1;
end
K1(i) = abs(vxt(i)*ayt(i) - axt(i)*vyt(i))/((vxt(i)^2+ vyt(i)^2)^1.5); % Continue-Method, cannot directly use on distert domain
Heading(i) = atan((y_2-y_1)/(x_2-x_1));
if Heading(i) <0
    Heading(i) = Heading(i) +pi;
end
 % Method (2)
%  [K2(i)] = curvature_cal(x_0,x_1,x_2,y_0,y_1,y_2);


    
    if sqrt(axt(i)^2+ayt(i)^2)>Acc_Threshold|| sqrt(jxt(i)^2+jxt(i)^2)>Jerk_Threshold || sqrt(vxt(i)^2+vyt(i)^2)>V_Threshold %|| max(K1)>Str_Threshold
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

figure(3)
plotyy(time(2:end),K1,time(2:end), Heading/pi*180 )
xlabel('Time[s]')
legend('Curvature[-]','Heading[deg]','location','best')

if T < T_Max
disp(['Final time is ', T])
else
    disp(['Fail to find solution'])
end

%% Save Data
Heading_in = [0,Heading];
K1_in = [0,K1];
Dis = zeros(1,length(xt));
Spiral_Msg = [xt ; yt; Heading_in ;K1_in; Dis]';
