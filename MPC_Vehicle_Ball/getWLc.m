function [ W, L, c] = getWLc(B, Xmax, Xmin, umax, umin, Gamma, Phi)
%GETWLC Summary of this function goes here
%   Detailed explanation goes here

nu = size(B,2);
nx = size(B,1);
N = size(Phi,1)/size(B,1);
           
Mi          = [zeros(nu,nx); 
               zeros(nu,nx);
               +eye(nx);
               -eye(nx)];
Ei          = [+eye(nu); -eye(nu); zeros(size(Mi,1)-2*nu,nu)];
bi          = [umax;
               -umin; 
               Xmax;
               -Xmin];

 MN = [eye(nx);-eye(nx)];
 bN = [Xmax;-Xmin];
% uncomment previous 2 lines to not use the terminal set constraints

Dcal = [Mi;repmat(0*Mi,N-1,1);0*MN];


Mcal = MN;
for i = 2:N
    Mcal = blkdiag(Mi,Mcal);
end
Mcal = [zeros(size(Mi,1),size(Mcal,2));Mcal];

Ecal = Ei;
for i = 2:N
    Ecal = blkdiag(Ecal,Ei);
end
Ecal = [Ecal;zeros(size(MN,1),size(Ecal,2))];

c = bN;
for i = 1:N
    c = [bi;c];
end

L = Mcal*Gamma+Ecal;
W = -Dcal-Mcal*Phi;

end

