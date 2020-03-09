function [theta_L,theta_R] = ackerman_steering(theta)
%Ackerman steering relation for steering wheels
% Turn right for Theta Positive 

wheel_width = 1.7;
l = 2.76; 
s1 =  wheel_width/2;
if theta ==0
theta_L = 0;
theta_R = 0;
else
R = l/tan(theta);
theta_L = atan(l/(R+s1));
theta_R =  atan(l/(R-s1));
end

end