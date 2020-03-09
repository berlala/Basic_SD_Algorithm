function point = fcn_rectangle(point1,point2, point3)
% give back a point to build a rectangle by the 4 points.
%point2 is the right-angle point

P_middle =  [(point1(1)+point3(1))/2,  (point1(2)+point3(2))/2];
vector =  [P_middle(1)-point2(1) , P_middle(2)-point2(2)];
point = P_middle + vector;
end