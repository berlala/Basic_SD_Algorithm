function Y=compute_angle(X,Xsum,n)
  for i=1:n+1
      deltaX(i)=Xsum(i,1)-X(1);
      deltaY(i)=Xsum(i,2)-X(2);
      r(i)=sqrt(deltaX(i)^2+deltaY(i)^2);
      if deltaX(i)>0
          theta=acos(deltaX(i)/r(i));
      else
          theta=pi-acos(deltaX(i)/r(i));
      end  
      Y(i)=theta;
  end
end