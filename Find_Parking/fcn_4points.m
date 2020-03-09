function cross  = fcn_4points(point_r,point1, point2, point3,point4)
% in this function, the output is the responding point for the Req point;
%[point1] and [point2] are the two line to consist of <line1>
%[point3] and [point4] are the two line to consist of <line2>
% [req] is a point on <Line1>
% this function give back a point that a <line3> which vertical to <Line1>
% by point [req] 

cross = [];

points = [point1(1), point1(2);
               point2(1), point2(2);
               point3(1), point3(2);
               point4(1), point4(2);];

if points(1,1) == points(2,1)
    y1 =0; % parallel to Y
    K2 = 0;
elseif points(1,2) == points(2,2)
    y1 = points(1,2); %parallel to X
    K1 = 0;
    x_cross = point_r(1);
    y_cross = ((x_cross - points(3,1))/(points(4,1) - points(3,1)))*(points(4,2) - points(3,2))+points(3,2);
   cross = [x_cross, y_cross];
else
    K1 = (points(2,2) - points(1,2))/(points(2,1) - points(1,1)); % the slope of the line1
    K2 = -1/K1; %the slope of the porpose line
    b  = point_r(2) - K2*point_r(1);

    %syms x
    %eqn1 = K2*x+ b ;
    %eqn2 = ((x - points(3,1))/(points(4,1) - points(3,1)))*(points(4,2) -
    %points(3,2))+points(3,2);
    %eqns = [eqn1-eqn2 == 0];
    %x_cross = eval(solve(eqns,x));
    
    x1 = points(3,1);
    x2 =  points(4,1);
    y1 =  points(3,2);
    y2 =  points(4,2);
    
    x_cross  = (x1*y1-x1*y2+x2*y1-x1*y1+x1*b-b*x2)/(K2*x2-K2*x1-y2+y1);
    y_cross  = x_cross*K2+b;
    
    cross = [x_cross, y_cross];
end
     
end