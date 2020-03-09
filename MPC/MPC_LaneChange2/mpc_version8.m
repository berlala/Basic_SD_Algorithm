clc;close all;clear all;
%%
% In this demo, MPC will follow a desire path.
% In this version, YAMLIP solver is used.
%% Generate the reference path
Td=0.02;% sample time
Freq = 1/Td

% %Reference Line 1
N=250;% number of the points on the path

Xref=zeros(N,4);
Tout=zeros(N,1);
v_ref = 2;
% for k=1:1:N
%     Xref(k,1)=k*Td*v_ref; %x
%     Xref(k,2)=0; %y
%     Xref(k,3)=v_ref; %v
%     Xref(k,4) = 0; % heading
%     Tout(k,1)=(k-1)*Td;
% end

% %Reference Line 3
% load ref_track_2.mat;
% Xref = route;
% for i = 1:length(Xref)
% Tout(i,1)=(i-1)*0.02;
% end

%Reference Line 2
% load I_Line.mat;
% N = length(Spiral_Msg(:,1));
% Xref = [Spiral_Msg(:,1),Spiral_Msg(:,2),Spiral_Msg(:,3)];
% Tout = Td*[0:1:N];

figure(1);
plot(Xref(:,1),Xref(:,2)); hold on;
xlabel('X'); ylabel('Y')
%% Tracking a constant reference trajectory
Nx=4;% Status Number  [X: x, y, v,psi]
Nu =2;% Input Number [U: a and Yaw Rate]
Horizon =5;
X00 = [0 0.2 0.1 0];% Initial Status
[Nr,Nc] =size(Xref); % Nr is the number of rows of Xref 100*3
% Mobile Robot Parameters
L = 1.74;%wheel base

a_last = 0.01; 
theta_last = 0;
v_last = X00(3);

x_real=zeros(Nr,Nc); % all real states
x_error=zeros(Nr,Nc); %all error
u_real=zeros(Nr,2); %all control input
u_error=zeros(Nr,2);
x_real(1,:)=X00; % initial State
x_error(1,:)=x_real(1,:)-Xref(1,:); %initial Error, Real Error
X_error=zeros(Nr,Nx*Horizon); % Error by Prediction
XXX=zeros(Nr,Nx*Horizon); % States by Prediction
% x,y,v,psi
Q=[0.01  0   0   0;
      0     0.6 0   0;
      0.0  0    0.01   0;
      0     0    0   0.6];

% a,theta
R = [0.2       0.0;
       0.0    0.01];
% form in YAMLIP

x = sdpvar(repmat(Nx,1,Horizon+1),repmat(1,1,Horizon+1));
u = sdpvar(repmat(Nu,1,Horizon),repmat(1,1,Horizon));
r = sdpvar(repmat(Nx,1,Horizon),repmat(1,1,Horizon));
d = sdpvar(1);
pastu = sdpvar(2,1);
oldu = [0,0];
%% MPC Process
%discrete kinematic model
for i=1:1:N
      [index,dis_nearest,X_clst] =  find_closest(Xref, x_real(i,:));
    %%
    sd = 0;
    XREF = zeros(Horizon,4);
    for j=1:Horizon
        x_ref = Xref(min(Nr,index+j),1);
        y_ref = Xref(min(Nr,index+j),2);
        sd = sd+  v_last*Td;
        inc = floor(sd/0.1); % how many steps may in the ref path?
        
        %Prediction XXX based on Prection Error
        XXX(i,1+Nx*(j-1))=X_error(i,1+Nx*(j-1))+Xref(min(Nr,index+inc),1); %first state:x
        XXX(i,2+Nx*(j-1))=X_error(i,2+Nx*(j-1))+Xref(min(Nr,index+inc),2); %second state:y
        XXX(i,3+Nx*(j-1))=X_error(i,3+Nx*(j-1))+Xref(min(Nr,index+inc),3); %third state:v
        XXX(i,4+Nx*(j-1))=X_error(i,4+Nx*(j-1))+Xref(min(Nr,index+inc),4); %third state:psi
        
        XREF(j,1) = Xref(min(Nr,index+inc),1);   
        XREF(j,2) = Xref(min(Nr,index+inc),2);
        XREF(j,3) = Xref(min(Nr,index+inc),3);   
        XREF(j,4) = Xref(min(Nr,index+inc),4);          
    end
    
   psi =X00(4); %ref psi
    theta_last = 0; %very important
    a=[1    0   cos(psi)*Td      -v_last*sin(psi)*Td ;
         0    1    sin(psi)*Td        v_last*cos(psi)*Td ;
         0    0     1                        0;
         0    0     tan(theta_last)/L*Td  1];
    b=[0    0
          0    0
          Td  0   
          0  v_last/(L*(cos(theta_last))^2)*Td];
     
 constraints = [-5<= diff([pastu u{:}]) <= 5];
objective = 0;
for k = 1:Horizon
    objective = objective  + u{k}'*R*u{k} +  x{k}'*Q*x{k}  ;
    constraints = [constraints, x{k+1} == a*x{k}+b*u{k}];
    constraints = [constraints, -10 <= u{k}<= 10, -5<=x{k+1}<=5];
end     
      
parameters_in = {x{1},[r{:}],pastu};
solutions_out = {[u{:}], [x{:}]};

controller = optimizer(constraints, objective,sdpsettings('solver','ipopt'),parameters_in,solutions_out);
inputs = {X00',XREF',oldu'};
[solutions,diagnostics] = controller{inputs};
U_e = solutions{1}(:,1);
oldu = U_e';
X = solutions{2};

u_error(i,1)=U_e(1,1);
u_error(i,2)=U_e(2,1);
    
    X00=x_real(i,:);
    a2=a_last+u_error(i,1);
    theta2=theta_last+u_error(i,2);
    
    if theta2 ==0
        theta2 = 0.0001;
    end
    
    %The REAL vehicle state
    XOUT=dsolve('Dx-v*cos(z)=0',...
                              'Dy-v*sin(z)=0',...
                              'Dv - a2 =0',...                             
                              'Dz-v/L*tan(theta2) =0',...
                              'x(0)=X00(1)',...
                              'y(0)=X00(2)',...
                              'v(0)=X00(3)',...
                              'z(0)=X00(4)');
     %tbu:state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT                     
    t=Td;
    x_real(i+1,1)=eval(XOUT.x);
    x_real(i+1,2)=eval(XOUT.y);
    x_real(i+1,3)=eval(XOUT.v);
    x_real(i+1,4)=eval(XOUT.z);

    if x_real(i+1,4)>2*pi
        x_real(i+1,4) = x_real(i+1,4) - 2*pi;
    else if x_real(i+1,4)<-2*pi
            x_real(i+1,4) = x_real(i+1,4) + 2*pi;
        end
    end

    if  abs((x_error(i,1))^2 +(x_error(i,2))^2 ) >0.2
        preview =0;
    else
        preview =1;
    end
    
    %if(i<Nr)
    % The REAL error between desire point on the path and real location
    x_error(i+1,:)=x_real(i+1,:)-Xref(min(Nr,index+preview),:); %preview control the error dynamic, which 
    % the error in the X direction lead the wave in velocity
    %end
    
    u_real(i,1)=a_last+u_error(i,1);
    a_last = u_real(i,1);
    u_real(i,2)=theta_last+u_error(i,2);
    theta_last = u_real(i,2);    
    v_last = x_real(i+1,3); 
    
    if a_last > 3
        a_last = 3;
    else if a_last < -3
            a_last =-3;
        end
    end
    if theta_last >pi/4 % maximum steering angle
        theta_last = pi/4;
    else if theta_last <-pi/4
            theta_last = -pi/4;
        end
    end
    
    figure(1);
    plot(x_real(i,1),x_real(i,2),'r*');
    title('Path Follow Result');
    xlabel('X');
    %axis([-1 8 -3 3]);
    ylabel('Y');
    axis equal;
    hold on;
    for k=1:1:Horizon
        X(i,k+1)=XXX(i,1+Nx*(k-1));
        Y(i,k+1)=XXX(i,2+Nx*(k-1));
    end
    X(i,1)=x_real(i,1);
    Y(i,1)=x_real(i,2);
    plot(X(i,:),Y(i,:),'g-.'); hold on
    plot(XREF(:,1),XREF(:,2),'k-.','linewidth',2);
    plot(X(i,1),Y(i,1),'go'); hold on
    
end
% figure(5)
% plot(X(2,:),Y(2,:),'b');
%% Plot
figure(2)
subplot(4,1,1);
plot(Tout(1:Nr),Xref(1:Nr,1),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,1),'k');
legend('Desired','Real')
ylabel('X')

subplot(4,1,2);
plot(Tout(1:Nr),Xref(1:Nr,2),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,2),'k');
legend('Desired','Real')
ylabel('Y')

subplot(4,1,3);
plot(Tout(1:Nr),Xref(1:Nr,4),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,4),'k');
legend('Desired','Real')
hold on;
xlabel('Time[s]')
ylabel('\theta')

subplot(4,1,4);
plot(Tout(1:Nr),Xref(1:Nr,3),'--','linewidth',2);
hold on;
plot(Tout(1:Nr),x_real(1:Nr,3),'k');
legend('Desired','Real')
hold on;
xlabel('Time[s]')
ylabel('Velocity[m/s]')


 
figure(3)
subplot(2,1,1);
plot(Tout(1:Nr),u_real(1:Nr,1));
ylabel('Acceleration[m/s^2]')
subplot(2,1,2)
plot(Tout(1:Nr),u_real(1:Nr,2));
xlabel('Time[s]');
ylabel('Theta[rad]')

%Error Plot
figure(4)
subplot(4,1,1);
plot(Tout(1:Nr),x_error(1:Nr,1),'k');
title('Overall Error')
%grid on;
ylabel('e(x)');
subplot(4,1,2);
plot(Tout(1:Nr),x_error(1:Nr,2),'k');
%grid on;
ylabel('e(y)');
subplot(4,1,3);
plot(Tout(1:Nr),x_error(1:Nr,3),'k');
%grid on;
ylabel('e(v)');
subplot(4,1,4);
plot(Tout(1:Nr),x_error(1:Nr,4),'k');
xlabel('Time[s]');
ylabel('e(\theta)');
