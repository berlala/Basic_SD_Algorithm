% Plot
%%
load Params.mat;
%%
x = posRef(:,1);
y = posRef(:,2);
z = posRef(:,3);
subplot(2,1,1)
plot(x,y)
subplot(2,1,2)
plot(yawRef)
