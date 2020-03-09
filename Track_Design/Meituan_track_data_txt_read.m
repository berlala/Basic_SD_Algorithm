%% Meituan Txt  data process
%%
load pathtestjul226.mat;
data = pathtestjul226;
%%
x = data(:,1);
y = data(:,2);
x0 = x(1);
y0 = y(1);
x = x - x0;
y= y -y0;

plot(x,y); hold on

Path_loc = [x,y,data(:,3)];
%%
i  = 1;
density = 0.1;
index = [1];
for i = 1:length(x)
    distance = (x(i)- x(index(end)))^2 +(y(i)-y(index(end)))^2;
    if distance >density^2
        index = [index;i];
        distance = 0;
    end
end
   
%% Filtered Path
Path = [x(index), y(index)];
plot(Path(:,1), Path(:,2),'o')

%% Calculate Curvature
K= [];
for i =1:length(Path)-9
a = Path(i,:);
b = Path(i+5,:);
c = Path(i+9,:);
S = ((b(1)-a(1))*(c(2)-a(2)) - (c(1)-a(1))*(b(2)-a(2)))/2;

A = distance_2point(a,b);
B = distance_2point(b,c);
C = distance_2point(c,a);

K(i) = S/(A*B*C);
end



    