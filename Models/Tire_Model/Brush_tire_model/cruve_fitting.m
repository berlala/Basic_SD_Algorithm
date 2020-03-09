%% Cruve fitting for Nonlinear Tire Model
%% 
% Bolin ZHAO
%%
load fiala_result.mat;
%%
x = F_y_log;
y = alpha_log;

% fit the invease function
p = polyfit(F_y_log, alpha_log, 6);

%%
x1 = linspace(-5000,5000);
y1 = polyval(p,x1);
plot(x,y);hold on
plot(x1, y1)

% The fitting result is bad.Using table for real-time useage;

