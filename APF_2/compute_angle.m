function Y=compute_angle(X,Xsum,n)%Y��������������x��ĽǶ�����,X��������꣬Xsum��Ŀ����ϰ�����������,��(n+1)*2����
  for i=1:n+1%n���ϰ���Ŀ
      deltaX(i)=Xsum(i,1)-X(1);
      deltaY(i)=Xsum(i,2)-X(2);
      r(i)=sqrt(deltaX(i)^2+deltaY(i)^2);
      theta=sign(deltaY(i))*acos(deltaX(i)/r(i));
      Y(i)=theta;   %angle;%����ÿ���Ƕ���Y�������棬��һ��Ԫ������Ŀ��ĽǶȣ����涼�����ϰ��ĽǶ�
  end