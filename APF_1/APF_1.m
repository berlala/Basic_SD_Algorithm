clear all;

Xo=[0 0];%
k=10;% Attract Gain factor
K=0;% For initialization
m=20;% Reject Gain factor
Po=20;% The Obstacle effect radius
l=0.2;% step size
J=100;%循环迭代次数

% Xsum=[10 10;
%               1 0.5;
%               2 0.5;
%               3 0.5;
%               4 0.5;
%               6 2;
%               5.5 5.5;
%               8 8.5;
%               8 7];
% Xsum=[7 7;
%               1 1;
%               2 1;
%               3 1;
%               4 1;
%               5 1;
%               6 1;
%               7 1];
Xsum = [10,10;
             3,2;
            3,3;
             5,7; 
             5.3,6;
             6,6;
             2,4;
             3,8;
             4,7;
             8,9];

n=length(Xsum)-1;% number of Obstacle

% The first one is the Object; the rest are Obstacle
Xj=Xo;%j=1
for j=1:J%循环开始
    Goal(j,1)=Xj(1);%
    Goal(j,2)=Xj(2);
%调用计算角度模块
   Theta=compute_angle(Xj,Xsum,n);%
%调用计算引力模块
   Angle=Theta(1);%
   angle_at=Theta(1);%
   [Fatx,Faty]=compute_Attract(Xj,Xsum,k,Angle,0,Po,n);%
   %Fatx, Faty is only a value 
   
    for i=1:n
       angle_re(i)=Theta(i+1);%
     end

    [Frerxx,Freryy,Fataxx,Fatayy]=compute_repulsion(Xj,Xsum,m,angle_at,angle_re,n,Po);%
    Fsumyj=Faty+Freryy+Fatayy;%
    Fsumxj=Fatx+Frerxx+Fataxx;%
    Position_angle(j)=atan(Fsumyj/Fsumxj);%
    
    Xnext(1)=Xj(1)+l*cos(Position_angle(j));
    Xnext(2)=Xj(2)+l*sin(Position_angle(j));
    Xj=Xnext;
    
    if ((Xj(1)-Xsum(1,1))>0)&((Xj(2)-Xsum(1,2))>0)%
       K=j;%
       break;
    end%
end%

Goal(j,1)=Xsum(1,1);%
Goal(j,2)=Xsum(1,2);

X=Goal(:,1);
Y=Goal(:,2);

x=Xsum(2:end, 1);
y=Xsum(2:end,2);
plot(x,y,'o',0,0,'ms',X,Y,'.-r');