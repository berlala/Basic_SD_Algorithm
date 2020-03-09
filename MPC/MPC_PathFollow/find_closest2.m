function [index,dis_nearest,X_clst] = find_closest2(RouteMapArray,X_state)
% Update 2018/10/22
% added the dis_nearest logic
% update 
% FF IAI Beijing, bolin.zhao@ff.com
xp = X_state(1);
yp = X_state(2);
dis = zeros(length(RouteMapArray(:,1)),1);
for ii =1 :length(RouteMapArray(:,1))
   dis_t = sqrt((RouteMapArray(ii,1)-xp)^2 + (RouteMapArray(ii,2)-yp)^2);
   dis(ii) = dis_t; 
end
index = find(dis == min(dis));
index = min(length(RouteMapArray)-1,max(2, index(1))); % The closest point on the defined Way-point
%% Calculate the Closest Distance

Index = [index-1, index, index+1];
height = [0,0]; psi_d = [0,0]; xa = [0,0]; ya = [0,0]; % necessary initilizatio for Simulink
for n =1:1:2
x1 = RouteMapArray(Index(n),1);
y1 = RouteMapArray(Index(n),2);
psi1 = RouteMapArray(Index(n),3);

x2 = RouteMapArray(Index(n+1),1);
y2 = RouteMapArray(Index(n+1),2);
psi2 = RouteMapArray(Index(n+1),3);

xa(n) = ((x2-x1)^2*xp +(y2-y1)^2*x1+(y2-y1)*(yp-y1)*(x2-x1))/((x2-x1)^2 + (y2-y1)^2);
ya(n) = (y2-y1)/(x2-x1)*(xa(n)-x1)+y1;

height(n) =  sqrt((xa(n)-xp)^2+ (ya(n)-yp)^2);

d1 = sqrt((xa(n)-x1)^2+ (ya(n)-y1)^2);
d2 = sqrt((x2-x1)^2+ (y2-y1)^2);
ratio = d1/d2;

psi_d(n)  = (psi2 - psi1)*ratio + psi1;
end

if height(1)>height(2)
    dis_nearest = height(2);
    X_clst = [xa(2),ya(2),psi_d(2)];
else
    dis_nearest = height(1);
    X_clst = [xa(1),ya(1),psi_d(1)];
end

end