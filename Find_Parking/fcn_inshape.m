function [in_shape] = fcn_inshape(point_r, points)
% judge whether Point_r is in the close shape consist of point1~4
sign_v = zeros(4,1);
in_shape = 0;
% points = [point1(1), point1(2);
%                point2(1), point2(2);
%                point3(1), point3(2);
%                point4(1), point4(2);];
           for i = 1:4
               if i+1>4
                   ii =1;
               else
                   ii = i+1;
               end
               vector_P = [points(i,1) - point_r(1), points(i,2) - point_r(2)];
               vector_Q = [points(ii,1) - point_r(1), points(ii,2) - point_r(2)];
               Corss_PQ = vector_P(1)* vector_Q(2) -   vector_P(2)* vector_Q(1); %vector cross
               sign_v(i) = sign(Corss_PQ);
           end
           in_shape = (sum(sign_v(:) == sign_v(1))==4); % the four signs are same
end