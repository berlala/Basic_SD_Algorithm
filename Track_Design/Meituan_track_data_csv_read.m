%% Track Plot
%%
clear;
%%
data_csv = csvread('single_frame_traj-20190723.csv',1,0);
x_init = data_csv(1,1);
y_init = data_csv(1,2);
x= 0; y =0;
%%
for i = 2:length(data_csv)
    x(i) = data_csv(i,1)-x_init;
    y(i) = data_csv(i,2)-y_init;    
end
%%
plot(x,y)

%%
route  = [x',y',data_csv(:,3), data_csv(:,4), data_csv(:,6)];
% x,y, theta[rad], curvature, speed

