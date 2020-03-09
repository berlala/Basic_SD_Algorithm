function [k] = points_curvature_2(Path)
% 3-points calculate curvature

a = Path(1,:);
b = Path(1+1,:);
c = Path(1+2,:);
S = ((b(1)-a(1))*(c(2)-a(2)) - (c(1)-a(1))*(b(2)-a(2)))/2;

A = distance_2point(a,b);
B = distance_2point(b,c);
C = distance_2point(c,a);

k = S/(A*B*C);
end
