clc;
close all;
clear all;
%%
% In this demo, MPC will follow a desire path.
%% Generate the reference path
N=100;% number of the points on the path
T=0.05;% sample time
% Xout=zeros(2*N,3);
% Tout=zeros(2*N,1);
Xout=zeros(N,3);
Tout=zeros(N,1);
for k=1:1:N
    Xout(k,1)=k*T;
    Xout(k,2)=2;
    Xout(k,3)=0;
    Tout(k,1)=(k-1)*T;
end
 
%% Tracking a constant reference trajectory
Nx=3;% Status Number  [X]
Nu =2;% Input Number [U: Vx and Yaw Rate]
Tsim =20;% Overall simulation time
X0 = [0 0 0];% Initial Status
[Nr,Nc] = size(Xout); % Nr is the number of rows of Xout，100*3
% Mobile Robot Parameters
c = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
L = 1.74;%wheel base
Rr = 1;
w = 1;
% Mobile Robot variable Model
vd1 = Rr*w; % For circular trajectory，参考系统的纵向速度
vd2 = 0;%参考系统的前轮偏角
 
%根据控制系统的维度信息，提前定义好相关矩阵并赋值
x_real=zeros(Nr,Nc);%X的真实状态
x_error=zeros(Nr,Nc);%X的误差状态
u_real=zeros(Nr,2);%真实控制量
u_error=zeros(Nr,2);%误差控制量
x_real(1,:)=X0;%初始状态
x_error(1,:)=x_real(1,:)-Xout(1,:);%与预期的误差值
X_error=zeros(Nr,Nx*Tsim);
XXX=zeros(Nr,Nx*Tsim);%用于保持每个时刻预测的所有状态值
q=[1 0 0;0 1 0;0 0 0.5];
Q_cell=cell(Tsim,Tsim);
for i=1:1:Tsim
    for j=1:1:Tsim
        if i==j
            Q_cell{i,j}=q;
        else 
            Q_cell{i,j}=zeros(Nx,Nx);
        end 
    end
end

Q=cell2mat(Q_cell);%权重矩阵
R=0.1*eye(Nu*Tsim,Nu*Tsim);%权重矩阵
 
%模型预测控制主体(discrete kinematic model)
for i=1:1:Nr
    t_d =Xout(i,3); %heading angle
    a=[1    0   -vd1*sin(t_d)*T;
       0    1   vd1*cos(t_d)*T;
       0    0   1;];
    b=[cos(t_d)*T   0;
       sin(t_d)*T   0;
       0            T;];     
    A_cell=cell(Tsim,1);
    B_cell=cell(Tsim,Tsim);
     for j=1:1:Tsim
        A_cell{j,1}=a^j;
        for k=1:1:Tsim
           if k<=j
                B_cell{j,k}=(a^(j-k))*b;
           else
                B_cell{j,k}=zeros(Nx,Nu);
           end
        end
    end
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
    
    H=2*(B'*Q*B+R);
    f=2*B'*Q*A*x_error(i,:)';
    A_cons=[];
    b_cons=[];
    lb=[-1;-1];
    ub=[1;1];
    tic
    [X,fval(i,1),exitflag(i,1),output(i,1)]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub);% qp solver
    toc
    X_error(i,:)=(A*x_error(i,:)'+B*X)';
    if i+j<Nr
         for j=1:1:Tsim
             XXX(i,1+3*(j-1))=X_error(i,1+3*(j-1))+Xout(i+j,1);
             XXX(i,2+3*(j-1))=X_error(i,2+3*(j-1))+Xout(i+j,2);
             XXX(i,3+3*(j-1))=X_error(i,3+3*(j-1))+Xout(i+j,3);
         end
    else
         for j=1:1:Tsim
             XXX(i,1+3*(j-1))=X_error(i,1+3*(j-1))+Xout(Nr,1);
             XXX(i,2+3*(j-1))=X_error(i,2+3*(j-1))+Xout(Nr,2);
             XXX(i,3+3*(j-1))=X_error(i,3+3*(j-1))+Xout(Nr,3);
         end
    end
    u_error(i,1)=X(1,1);
    u_error(i,2)=X(2,1);
    Tvec=[0:0.05:4];
    X00=x_real(i,:);
    vd11=vd1+u_error(i,1);
    vd22=vd2+u_error(i,2);
    XOUT=dsolve('Dx-vd11*cos(z)=0','Dy-vd11*sin(z)=0','Dz-vd22=0','x(0)=X00(1)',...
        'y(0)=X00(2)','z(0)=X00(3)');
     t=T; 
     x_real(i+1,1)=eval(XOUT.x);
     x_real(i+1,2)=eval(XOUT.y);
     x_real(i+1,3)=eval(XOUT.z);
     if(i<Nr)
         x_error(i+1,:)=x_real(i+1,:)-Xout(i+1,:);
     end
    u_real(i,1)=vd1+u_error(i,1);
    u_real(i,2)=vd2+u_error(i,2);
    
    figure(1);
    plot(Xout(1:Nr,1),Xout(1:Nr,2));
    hold on;
    plot(x_real(i,1),x_real(i,2),'r*');
    title('Path Follow Result');
    xlabel('X');
    axis([-1 5 -1 3]);
    ylabel('Y');
    hold on;
    for k=1:1:Tsim
         X(i,k+1)=XXX(i,1+3*(k-1));
         Y(i,k+1)=XXX(i,2+3*(k-1));
    end
    X(i,1)=x_real(i,1);
    Y(i,1)=x_real(i,2);
    plot(X(i,:),Y(i,:),'y.')
    hold on;
    
end
% figure(5)
% plot(X(2,:),Y(2,:),'b');
%% Plot
figure(2)
subplot(3,1,1);
plot(Tout(1:Nr),Xout(1:Nr,1),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,1),'k');
legend('Desired','Real')
%grid on;
%title('状态量-横向坐标X对比');
ylabel('X')
subplot(3,1,2);
plot(Tout(1:Nr),Xout(1:Nr,2),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,2),'k');
legend('Desired','Real')
%grid on;
%title('状态量-横向坐标Y对比');
ylabel('Y')
subplot(3,1,3);
plot(Tout(1:Nr),Xout(1:Nr,3),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,3),'k');
legend('Desired','Real')
%grid on;
hold on;
%title('状态量-\theta对比');
xlabel('Time[s]')
ylabel('\theta')
 
figure(3)
subplot(2,1,1);
plot(Tout(1:Nr),u_real(1:Nr,1));
%grid on;
%title('控制量-纵向速度v对比');
ylabel('Vx')
subplot(2,1,2)
plot(Tout(1:Nr),u_real(1:Nr,2));
%grid on;
%title('控制量-角加速度对比');
xlabel('Time[s]');
ylabel('Yaw Rate[rad/s]')

figure(4)
subplot(3,1,1);
plot(Tout(1:Nr),x_error(1:Nr,1),'k');
title('Overall Error')
%grid on;
ylabel('e(x)');
subplot(3,1,2);
plot(Tout(1:Nr),x_error(1:Nr,2),'k');
%grid on;
ylabel('e(y)');
subplot(3,1,3);
plot(Tout(1:Nr),x_error(1:Nr,3),'k');
%grid on;
xlabel('Time[s]');
ylabel('e(\theta)');
