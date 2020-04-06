% Roll_Over Condition Check
%%
% under assumption of (1)
w = 0.88;
h = 0.764;
ay_max = 9.8*w/(2*h);
theta = 0:0.01:30/180*3.14;
v = [];
for i = 1: length(theta)
v(i) = sqrt(ay_max*1.5/theta(i));
end

sw_ratio = 15.4;
plot(v*3.6, theta*sw_ratio/3.14*180);
xlabel('Velocity[km/h]')
ylabel('SWA[deg]')
axis([0,60,0,270])
grid on