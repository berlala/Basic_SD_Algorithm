function [dist] = fcn_dist(point1,point2)
x1 = point1(1);
y1 = point1(2);

x2 = point2(1);
y2 = point2(2);

dist = sqrt((x1-x2)^2 + (y1-y2)^2);
end