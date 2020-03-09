function inline = fcn_inline(point1, point2, point3)
% judge whether the point1 on the line which consist of point2 and 3. And
% in the middle of them.
dist_12 = fcn_dist(point1, point2);
dist_13 = fcn_dist(point1, point3);
dist_23 =  fcn_dist(point2, point3);

if dist_12/dist_23<1 &&dist_23/dist_23<1
    inline = 1;
else
    inline = 0;
end
end