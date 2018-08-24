clear all;

Xo=[0 0];%
k=15;%
K=0;%
m=5;%
Po=5.5;%
s=0.2;%
a=0.5;
l=0.1;%
J=500;%

Xsum=[7 7;1 1;2 1;3 1;4 1;5 1;6 1;7 1];
n=length(Xsum)-1;% number of Obstacle

Xj=Xo;
for j=1:J%
    Goal(j,1)=Xj(1);%
    Goal(j,2)=Xj(2);

   Theta=compute_angle(Xj,Xsum,n);%
   Angle=Theta(1);%
   angle_at=Theta(1);%
   [Fatx,Faty]=compute_Attract(Xj,Xsum,k,Angle,0,Po,n);% in fact only calculate the Attract Force between Object point
   %
   
    for i=1:n
       angle_re(i)=Theta(i+1);%
     end

    [Frerxx,Freryy,Fataxx,Fatayy]=compute_repulsion(Xj,Xsum,m,angle_at,angle_re,n,Po,a);%
    Fsumyj=Faty+Freryy+Fatayy;%y
    Fsumxj=Fatx+Frerxx+Fataxx;%x
    Position_angle(j)=atan(Fsumyj/Fsumxj);%

%    Xnext(1)=Xj(1)+l*cos(Position_angle(j));
%    Xnext(2)=Xj(2)+l*sin(Position_angle(j));
    RF=sqrt(Fsumyj^2+Fsumxj^2);
    Xnext(1)=Xj(1)+l*Fsumxj/RF;
    Xnext(2)=Xj(2)+l*Fsumyj/RF;
    Xj=Xnext;
    if (abs(Xj(1)-Xsum(1,1))<0.1)&(abs(Xj(2)-Xsum(1,2))<0.1)%
       break;
    end%
end%
K=j;
Goal(K,1)=Xsum(1,1);
Goal(K,2)=Xsum(1,2);

X=Goal(:,1);
Y=Goal(:,2);

x=Xsum(2:end, 1);
y=Xsum(2:end,2);
plot(x,y,'o',0,0,'ms',X,Y,'.-r');