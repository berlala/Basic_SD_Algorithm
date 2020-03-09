function [Fx,Fy,Mz,kappa,alpha,omega_dot]=brush_tyre_combine(Vx,Vsy,Fz,Md,Mb,omega)
% condition: cpx = cpy =cp, mu_x = mu_y = mu

% omega: half-drive-axis rotation speed [rad/s]
% Vx: real speed [m/s]

rf=0.3;
cp=9e6;
mu=1.2;
cz=250000; % vertical 
Ip=2;
eps=0.000001; %% to avoid the deadlock

rho=max(0,Fz/cz); % deflection on the Z-axis by mass load
r=rf;%%only in this place, the tyre will be rigid
a=0.35*rf*(rho/rf+2.25*sqrt(rho/rf));%% P126 Empirical formula, contact length one side

Vr=omega*rf;  % desired speed from powertrain on tire
Vsx=Vx-Vr; % longitude slid speed

kappa=-Vsx/abs(Vx+eps);
alpha= -atan2(-Vsy,Vx);
% sigmax = kappa/(1+kappa);
% Vx = Vr+Vsx;

sigmax=-Vsx/abs(Vr);
sigmay=-Vsy/abs(Vr);
sigma=sqrt(sigmax^2+sigmay^2+eps);

Theta=2*cp*a^2/(3*mu*Fz+eps); 
sigmasl=1/Theta;

  if sigma< sigmasl
    F=mu*Fz*Theta*sigma*(3-3*abs(Theta*sigma)+(Theta*sigma)^2); 
    t=1/3*a*(1-3*abs(Theta*sigma)+3*abs(Theta*sigma)^2-abs(Theta*sigma)^3)/(1-abs(Theta*sigma)+1/3*abs(Theta*sigma)^2);
    
        else
        F=mu*Fz; % pure slide
        t=0;   
  end
         
    Fx=F*sigmax/sigma;
    Fy=F*sigmay/sigma;
    Mz=-t*Fy;
    
    
omega_dot=(-Fx*r+Md-Mb)/Ip;

return
