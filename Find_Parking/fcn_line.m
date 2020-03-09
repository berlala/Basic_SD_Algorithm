function y = fcn_line(x, point1, point2)
% Two point method 
%point1 = [x1,y1];
%point2 = [x2, y2];
if point1(1) == point2(1) %vertical to X
    y = NaN; % in this case, y can be any
elseif point1(2) == point2(2)  %paralle to X
    y = point(2); 
else
    y = ((x - point2(1))/(point1(1) - point2(1)))*(point1(2) - point2(2))+point2(2);
end
end