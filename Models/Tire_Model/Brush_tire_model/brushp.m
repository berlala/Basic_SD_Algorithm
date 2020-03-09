function [Fx,Fy,Mz,t] = brushp(kappa,alpha,Fz)
% pure slip model 
cpx=9000000;
cpy=9000000;
mu=1.2;

cz=250000;
rho=Fz/cz;
rf=0.3;
r=rf-rho;
a=0.35*rf*(rho/rf+2.25*sqrt(rho/rf));

Thetay=2*cpy*a^2/(3*mu*Fz);
alphasl=atan(1/Thetay);
sigmay=tan(alpha);

Thetax=2*cpx*a^2/(3*mu*Fz);
kappasl=-1/(1+Thetax);%% why there is 2 kappasl
%kappasl=-1/(1-Thetax);
sigmax=kappa/(1+kappa);


   if abs(alpha)< alphasl
   Fy=3*mu*Fz*Thetay*sigmay*(1-abs(Thetay*sigmay)+1/3*(Thetay*sigmay)^2);
    t=1/3*a*(1-3*abs(Thetay*sigmay)+3*abs(Thetay*sigmay)^2-abs(Thetay*sigmay)^3)/(1-abs(Thetay*sigmay)+1/3*abs(Thetay*sigmay)^2);
   else
        Fy=mu*Fz*abs(alpha)/alpha;%abs(alpha)/alpha to get the sign +or-
        t=0;   
   end
   
   if abs(kappa)< abs(kappasl)% small than kappasl means in linear region
    Fx=2*cpx*a^2*kappa;
   else
            Fx=mu*Fz*abs(kappa)/kappa; 
         % kappa is small , in linear region 
   end
    Mz=-t*Fy;
    return
    