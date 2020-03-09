x= 50:0.2:200;
y = 4.175*ones(1,length(x));
psi = zeros(1,length(x));
cur = psi;
dis = [48.112862] + x-50;

extend = [x;y;psi;cur;dis]';

Spiral_Msg = [Spiral_Msg;  extend;];

plot(Spiral_Msg(:,1),Spiral_Msg(:,2))