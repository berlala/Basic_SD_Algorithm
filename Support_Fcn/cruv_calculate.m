function k=cruv_calculate(x,y,dt) 
%paramater formula for cruvature calculation
h=dt;
y1=gradient(y,h);
y2=2*2*del2(y,h);
x1=gradient(x,h);
x2=2*2*del2(x,h);
k=(y2.*x1-y1.*x2)./((x1.^2+y1.^2).^(3/2));
