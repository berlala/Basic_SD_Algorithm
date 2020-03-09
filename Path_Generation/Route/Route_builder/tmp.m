% 定义点集[x,y,theta,curv]
point_array=zeros(30,4);
point_num=1;
%设计x
x=0;y=0;theta=0;curv=0;
randx=0;randy=0;randtheta=0;randcurv=0;
for x=-800:140:800
%计算y
 y=sqrt(800*800-x*x);
 %计算 theta
 theta=atan2(y,x);
 theta=theta-pi/2;
 while theta>pi/2
     theta=theta-pi;
 end
 while theta<-pi/2
     theta=theta+pi;
 end
 %第一个点不做随机
 if point_num==1
     point_array(point_num,1)=x;
     point_array(point_num,2)=y;
     point_array(point_num,3)=theta;
     point_array(point_num,4)=0;
 else%跟新x
      bflag=1;
     while(bflag)       
         bflag=0;
         point_array(point_num,1)=x+randi([-20,20],1);
         point_array(point_num,2)=y+randi([-40,40],1);
         point_array(point_num,3)=theta+randi([-50,50],1)*0.1*pi/180;
         point_array(point_num,4)=randi([-15,15],1)*0.01;
         %生成，验证是否重新生成
         
         %
     end
 end
     point_num=point_num+1;
end%x end
%第二段
for x=point_array(point_num-1,1):-140:-800
%计算y
 y=-sqrt(800*800-x*x);
 %计算 theta
 theta=atan2(y,x);
 theta=theta-pi/2;
 while theta>-pi/2
     theta=theta-pi;
 end
 while theta<-3*pi/2
     theta=theta+pi;
 end
 %第一个点不做随机
%跟新x
      bflag=1;
     while(bflag)       
         bflag=0;
         point_array(point_num,1)=x+randi([-20,20],1);
         point_array(point_num,2)=y+randi([-40,40],1);
         point_array(point_num,3)=theta+randi([-50,50],1)*0.1*pi/180;
         point_array(point_num,4)=randi([-15,15],1)*0.01;
         %生成，验证是否重新生成
         
         %
     end
     point_num=point_num+1;
end%x end

%绘制生成随机点
figure(1);
hold on;grid on;
for i=1:1:point_num-1
    %clf;
    hold on;grid on;
    plot(point_array(i,1),point_array(i,2),'r*','Markersize',10);
    %绘制当前点的车体形状
    carsize=[-1.1,4.3,4.3,-1.1,-1.1;1.2,1.2,-1.2,-1.2,1.2]; 
    x_coor=point_array(i,1);
    y_coor=point_array(i,2);
    theta=point_array(i,3);
    bound_xy=carsize;
    sin_val=sin(-theta);
    cos_val=cos(-theta);
    for zz=1:1:length(carsize(1,:))           
         bound_xy(1,zz)=carsize(1,zz)*cos_val+carsize(2,zz)*sin_val+x_coor;
         bound_xy(2,zz)=-carsize(1,zz)*sin_val+carsize(2,zz)*cos_val+y_coor;
     end
     for zz=1:1:length(carsize(1,:))-1
        plot([bound_xy(1,zz) bound_xy(1,zz+1)], [bound_xy(2,zz) bound_xy(2,zz+1)],'g','linewidth',3);
     end
     plot([bound_xy(1,1) bound_xy(1,4)], [bound_xy(2,1) bound_xy(2,4)],'g','linewidth',3);
     pause(0.1);
end


