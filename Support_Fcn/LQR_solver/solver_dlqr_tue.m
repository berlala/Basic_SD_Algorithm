function U_LQR = solver_dlqr_tue(A,B,Q,R,X)

% TU/e LQR solver for optimal control 

n =100;
% PHI = repmat(A,n,1);
PHI = zeros(4*n, 4);
for i=2:n
PHI((4*i-3):4*i,:) = A^n;
end
TAO = zeros(4*n,n);
for i=1:n
for j=1:i
TAO((4*i-3):4*i,j) = A^(i-j)*B;
end
end
%y_hat = polyval(coeffs, (0.1:0.1:1)*U)';

OMEGA = zeros(4*n, 4*n);
for i=1:n% P =Q
OMEGA((4*i-3):4*i, (4*i-3):4*i) = Q;
end
PSI = zeros(n, n);
for i =1:n
PSI(i, i) = R;
end

G = 2*(PSI+TAO'*OMEGA*TAO);
F2 = 2*TAO'*OMEGA*PHI;
G_LQR = -inv(G)*F2*X;
U_LQR = G_LQR(1);