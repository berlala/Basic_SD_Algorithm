function dis = distance_2point(a,b)
% a = [x1,y1];  b = [x2, y2]
%%
x1 = a(1);
x2 = b(1);
y1 = a(2);
y2 = b(2);

dis = sqrt((x1-x2)^2 + (y1-y2)^2); 
end
