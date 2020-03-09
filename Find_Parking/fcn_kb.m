function [k,b] = fcn_kb(point1, point2)
% Two point method 
%point1 = [x1,y1];
%point2 = [x2, y2];
% the line : y = kx+b
if point1(1) == point2(1) %vertical to X
    k = NaN; % in this case, y can be any
    b = NaN;
elseif point1(2) == point2(2)  %paralle to X
    k = 0; 
    b = point1(2);
else
    k =  (point2(2) - point1(2))/(point2(1) - point1(1));
    KK = -1/K; %the slope of the porpose line
    b  = point1(2) - KK*point1(1);
end
end