function fcn_bezier( vertices )

% e,g, vertices = [     0     0;
                                 % 10    15;
                                 % 20     2]'; % pay attention in T
                                % if you want the 3D, the input should like
                                % [1,1,1; 2,2 ,2 ; 3,5,8;]
 
Dim=size(vertices,1);% Dim can only be 2 or 3 in this version 
NumPoint=size(vertices,2)-1;% Number of the control point
t=0:0.001:1;
x=[];y=[];z=[];
if Dim==2
     x=(1-t).^(NumPoint)*vertices(1,1);
     y=(1-t).^(NumPoint)*vertices(2,1);
  for j=1:NumPoint
      w=factorial(NumPoint)/(factorial(j)*factorial(NumPoint-j))*(1-t).^(NumPoint-j).*t.^(j);
      x=x+w*vertices(1,j+1);
      y=y+w*vertices(2,j+1);
  end
plot(vertices(1,:),vertices(2,:),'b');
hold on;grid on;
axis tight;  
xlabel('X');ylabel('Y');
plot(x,y,'r');
end
 
if Dim==3
    x=(1-t).^(NumPoint)*vertices(1,1);
    y=(1-t).^(NumPoint)*vertices(2,1);
    z=(1-t).^(NumPoint)*vertices(3,1);
 for j=1:NumPoint
     w=factorial(NumPoint)/(factorial(j)*factorial(NumPoint-j))*(1-t).^(NumPoint-j).*t.^(j);
     x=x+w*vertices(1,j+1);
     y=y+w*vertices(2,j+1);
     z=z+w*vertices(3,j+1);
 end
plot3(vertices(1,:),vertices(2,:),vertices(3,:),'b');
hold on;grid on;
axis tight;  
%axis([0.5,1.5,0.5,1.5,0,0.7]);
xlabel('X');ylabel('Y');zlabel('Z');
plot3(x,y,z,'r');
view(3);
end
 
end