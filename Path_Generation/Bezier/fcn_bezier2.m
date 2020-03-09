function [X,Y]=fcn_bezier2(x,y)
%用法：
%bezier(x,y)
%       生成n-1次贝塞尔曲线，其中x和y是n个点的坐标
%h=bezier(x,y)
%       生成n-1次贝塞尔曲线并返回曲线句柄
%[X,Y]=bezier(x,y)
%       返回n-1次贝塞尔曲线的坐标
%例子：
%bezier([5,6,10,12],[0 5 -5 -2])
n=length(x);
t=linspace(0,1);
xx=0;yy=0;
for k=0:n-1
    tmp=nchoosek(n-1,k)*t.^k.*(1-t).^(n-1-k);
    xx=xx+tmp*x(k+1);
    yy=yy+tmp*y(k+1);
end
if nargout==2
    X=xx;Y=yy;
end
% h=plot(xx,yy);
if nargout==1
    X=h;
end