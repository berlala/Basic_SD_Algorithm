%% Bicycle Model Test
%%

%clear;

%x = 0;
%y = -1.3;
%phi = pi/2;
v_x = vx_0;
x = x_0;
y = y_0;
phi = phi_0;


for i = 1: length(theta_all)
    theta = theta_all(i);
  [x_1, y_1, phi_1, v_1] = bicycle_model(x, y, phi,v_x, theta, Ts);
    
  x_log(i) = x_1;
  y_log(i) = y_1;
  phi_log(i) = phi_1;
  v_log(i) = v_1;
  
  x = x_1;
  y = y_1;
  phi = phi_1;
  v = v_1;
end

%%
figure(1)
plot(x_log, y_log);hold on
xlabel('X[m]')
ylabel('Y[m]')
%legend('Bicycle 2m/s', 'Bicycle 5m/s', 'Bicycle 10m/s', 'Bicycle 15m/s')