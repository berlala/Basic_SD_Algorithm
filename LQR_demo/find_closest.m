function [index,dis_nearest] = find_closest(RouteMapArray,X_state)
% Update 2018/09/28
% added the dis_nearest logic
% FF IAI Beijing, bolin.zhao@ff.com
x = X_state(1);
y = X_state(2);
dis = zeros(length(RouteMapArray(:,1)),1);
for ii =1 :length(RouteMapArray(:,1))
   dis_t = sqrt((RouteMapArray(ii,1)-x)^2 + (RouteMapArray(ii,2)-y)^2);
   dis(ii) = dis_t; 
end

index = find(dis == min(dis));
index = index(1);
dis_nearest = min(dis);
end