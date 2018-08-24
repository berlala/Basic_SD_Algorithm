function [Theta, X_ref, Y_ref]   = Pure_pursuit_control( X_c, Y_c,L,  Yaw, t)
l =2.5; % wheelbase
%L = 1; % look ahead distance
X_ref = 20;%1*t;
Y_ref = 20;

P_e = sqrt((X_ref-X_c)^2 + (Y_ref - Y_c)^2);

p = polyfit([ X_c, X_ref], [Y_c, Y_ref],1);
if p(1) < 0 
     P_e = P_e*-1;% Turn right
end

Alfa = atan((Y_ref - Y_c)/(X_ref-X_c)) - Yaw;

%Theta = atan((2*l*P_e)/L^2);

Theta = atan((2*L*sin(Alfa))/l); %BJ GongYe Uni

if Theta > 1.5375 % constrain by the steering wheel
    Theta = 1.5375;
elseif Theta < -1.5375
   Theta = -1.5375;
end

return