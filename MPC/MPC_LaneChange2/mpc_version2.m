clc;
close all;
clear all;
%%
% In this demo, MPC will follow a desire path.
%% Generate the reference path
Td=0.05;% sample time

% %Reference Line 1
% N=100;% number of the points on the path
% % Xout=zeros(2*N,3);
% % Tout=zeros(2*N,1);
% Xout=zeros(N,3);
% Tout=zeros(N,1);
% 
% for k=1:1:N
%     Xout(k,1)=k*Td;
%     Xout(k,2)=2;
%     Xout(k,3)=0;
%     Tout(k,1)=(k-1)*Td;
% end

%Reference Line 2
load I_Line.mat;
N = length(Spiral_Msg(:,1));
Xout = [Spiral_Msg(:,1),Spiral_Msg(:,2),Spiral_Msg(:,3)];
Tout = Td*[0:1:N];

%% Tracking a constant reference trajectory
Nx=3;% Status Number  [X]
Nu =2;% Input Number [U: Vx and Yaw Rate]
Horizon =50;
X0 = [0 -3 0];% Initial Status
[Nr,Nc] =size(Xout); % Nr is the number of rows of Xout 100*3
% Mobile Robot Parameters
L = 1.74;%wheel base
%v_ref = 1; % For circular trajectory
v_last = 1; % v_ref from the trajectory or Spd setting
theta_last = 0;
 
x_real=zeros(Nr,Nc); % all real states
x_error=zeros(Nr,Nc); %all error
u_real=zeros(Nr,2); %all control input
u_error=zeros(Nr,2); 
x_real(1,:)=X0; % initial State
x_error(1,:)=x_real(1,:)-Xout(1,:); %initial Error, Real Error
X_error=zeros(Nr,Nx*Horizon); % Error by Prediction
XXX=zeros(Nr,Nx*Horizon); % States by Prediction
q=[1 0 0;
      0 1 0;
      0 0 0.5];
Q_cell=cell(Horizon,Horizon);

for i=1:1:Horizon
    for j=1:1:Horizon
        if i==j
            Q_cell{i,j}=q;
        else 
            Q_cell{i,j}=zeros(Nx,Nx);
        end 
    end
end 

Q=cell2mat(Q_cell); %60*60 for 3 status in 20 seconds

r = [1 0;
         0 1];
 R_cell=cell(Horizon,Horizon);

for i=1:1:Horizon
    for j=1:1:Horizon
        if i==j
            R_cell{i,j}=r;
        else 
            R_cell{i,j}=zeros(Nu,Nu);
        end 
    end
end
R=cell2mat(R_cell); 
%% MPC Process
%discrete kinematic model
for i=1:1:Nr
    %t_d =Xout(i,3); 
    t_d = 0;
    a=[1    0   -v_last*sin(t_d)*Td
          0    1     v_last*cos(t_d)*Td
          0    0     1];
    b=[cos(t_d)*Td   0
           sin(t_d)*Td    0
                    0         Td];
    A_cell=cell(Horizon,1);
    B_cell=cell(Horizon,Horizon);
    for j=1:1:Horizon
        A_cell{j,1}=a^j;
        for k=1:1:Horizon
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
    lb=[-2;-60/180*pi]; %input lower boundary
    ub=[2; 60/180*pi]; %input upper boundary

    tic
    [U_e,fval(i,1),exitflag(i,1),output(i,1)]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub);% qp solver
    toc
    
    X_error(i,:)=(A*x_error(i,:)'+B*U_e)';
    if i+j<Nr
        for j=1:1:Horizon
            XXX(i,1+3*(j-1))=X_error(i,1+3*(j-1))+Xout(i+j,1);
            XXX(i,2+3*(j-1))=X_error(i,2+3*(j-1))+Xout(i+j,2);
            XXX(i,3+3*(j-1))=X_error(i,3+3*(j-1))+Xout(i+j,3);
        end
    else
        for j=1:1:Horizon
            XXX(i,1+3*(j-1))=X_error(i,1+3*(j-1))+Xout(Nr,1);
            XXX(i,2+3*(j-1))=X_error(i,2+3*(j-1))+Xout(Nr,2);
            XXX(i,3+3*(j-1))=X_error(i,3+3*(j-1))+Xout(Nr,3);
        end
    end
    u_error(i,1)=U_e(1,1);
    u_error(i,2)=U_e(2,1);

    X00=x_real(i,:);
    vdx1=v_last+u_error(i,1);
    theta2=0+u_error(i,2);
    XOUT=dsolve('Dx-vdx1*cos(z)=0',...
                             'Dy-vdx1*sin(z)=0',...
                             'Dz-theta2=0',...
                             'x(0)=X00(1)',...
                             'y(0)=X00(2)',...
                             'z(0)=X00(3)');
    t=Td;
    x_real(i+1,1)=eval(XOUT.x);
    x_real(i+1,2)=eval(XOUT.y);
    x_real(i+1,3)=eval(XOUT.z);
    if(i<Nr)
        x_error(i+1,:)=x_real(i+1,:)-Xout(i+1,:);
    end
    u_real(i,1)=v_last+u_error(i,1);
    v_last = u_real(i,1);
    u_real(i,2)=0+u_error(i,2);
    theta_last = u_real(i,2);
    
    if theta_last >pi
        theta_last = 2*pi - theta_last ;
    else if theta_last <-pi
            theta_last = theta_last+2*pi;
        end
    end
    
       
    figure(1);
    plot(Xout(1:Nr,1),Xout(1:Nr,2));
    hold on;
    plot(x_real(i,1),x_real(i,2),'r*');
    title('Path Follow Result');
    xlabel('X');
    %axis([-1 8 -3 3]);
    ylabel('Y');
    hold on;
    for k=1:1:Horizon
        X(i,k+1)=XXX(i,1+3*(k-1));
        Y(i,k+1)=XXX(i,2+3*(k-1));
    end
    X(i,1)=x_real(i,1);
    Y(i,1)=x_real(i,2);
    plot(X(i,:),Y(i,:),'y-.')
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

ylabel('X')
subplot(3,1,2);
plot(Tout(1:Nr),Xout(1:Nr,2),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,2),'k');
legend('Desired','Real')

ylabel('Y')
subplot(3,1,3);
plot(Tout(1:Nr),Xout(1:Nr,3),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,3),'k');
legend('Desired','Real')
hold on;

xlabel('Time[s]')
ylabel('\theta')
 
figure(3)
subplot(2,1,1);
plot(Tout(1:Nr),u_real(1:Nr,1));

ylabel('Vx')
subplot(2,1,2)
plot(Tout(1:Nr),u_real(1:Nr,2));

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
