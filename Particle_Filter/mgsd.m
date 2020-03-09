function P_out  = mgsd(x,y,ux,uy)
% Multivariate-Gaussian's standard deviation for Paticle Filter

% x,y is the nearest measurement 
% ux, uy is the location of landmark from map
%
%
std_x = 0.3;
std_y = 0.3;

P  = 1/(2*pi*std_x*std_y)*(exp(1))^(-((x-ux)^2/(2*std_x^2)+(y-uy)^2/(2*std_y^2)));
P_out = vpa(P);

end