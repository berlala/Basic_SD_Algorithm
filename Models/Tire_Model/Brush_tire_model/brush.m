function [Fx,Fy,Mz,t] = brush(kappa,alpha,Fz)
% combined slip model 
cp=9000000;
mu=1.2;

cz=250000;
rho=Fz/cz;
rf=0.3;
r=rf-rho;
a=0.35*rf*(rho/rf+2.25*sqrt(rho/rf));%% P126 Empirical formula

Theta=2*cp*a^2/(3*mu*Fz); 
sigmasl=1/Theta;
sigmax=kappa/(1+kappa);
sigmay=tan(alpha)/(1+kappa);
sigma=sqrt(sigmax^2+sigmay^2);


   if sigma< sigmasl
    F=mu*Fz*Theta*sigma*(3-3*abs(Theta*sigma)+(Theta*sigma)^2); 
    t=1/3*a*(1-3*abs(Theta*sigma)+3*abs(Theta*sigma)^2-abs(Theta*sigma)^3)/(1-abs(Theta*sigma)+1/3*abs(Theta*sigma)^2);
        else
        F=mu*Fz;
        t=0;   
   end
    Fx=F*sigmax/sigma;
    Fy=F*sigmay/sigma;
    Mz=-t*Fy;
    
return