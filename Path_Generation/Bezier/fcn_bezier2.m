function [X,Y]=fcn_bezier2(x,y)
%�÷���
%bezier(x,y)
%       ����n-1�α��������ߣ�����x��y��n���������
%h=bezier(x,y)
%       ����n-1�α��������߲��������߾��
%[X,Y]=bezier(x,y)
%       ����n-1�α��������ߵ�����
%���ӣ�
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