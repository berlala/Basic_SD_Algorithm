function cross  = fcn_3points(point1, point2, point3)
%point2 and point3 are the point on the line
%cross is the coordinate of the corss point

if point3(1) == point2(1) % vertical case
    cross = [point2(1), point1(2)];
elseif point3(2)  == point2(2)
    cross = [point1(1),point2(2)]; % level case
else
    K = (point3(2) - point2(2))/(point3(1) - point2(1)); % the slope of the line
    KK = -1/K; %the slope of the porpose line
    b  = point1(2) - KK*point1(1);
     
%    syms x
%     eqn1 = KK*x+ b ;
%     eqn2 = ((x - point3(1))/(point2(1) - point3(1)))*(point2(2) - point3(2))+point3(2);
%     eqns = [eqn1-eqn2 == 0];
%     %eqns = [KK*x+ b,  ((x - point3(1))/(point2(1) - point3(1)))*(point2(2) - point3(2))+point3(2)];
%     x_cross = eval(solve(eqns,x));
    x1 = point3(1);
    x2 =  point2(1);
    y1 =  point3(2);
    y2 =  point2(2);
    
    x_cross  = (x1*y1-x1*y2+x2*y1-x1*y1+x1*b-b*x2)/(KK*x2-KK*x1-y2+y1);
    
    y_cross  = x_cross*KK+b;
    
    cross = [x_cross, y_cross];
end
end