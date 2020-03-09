function [U_lqr, K_0] = solver_dlqr(A,B,Q,R,X)
% Bolin ZHAO, Refer Standford EE 363
% for discrete LQR
%Q =eye(states number)
%R= eye(input number)
%%if you want to check the result, compare with dlqr() function
%%
%U_lqr: the control output
% N: required loop number, at least 500
%%
P_n = Q;
N =1000;

P_t = zeros(size(A,1)*N,size(A,2));
for n = 1:N
    P = Q+A'*P_n*A - A'*P_n*B*inv(R+B'*P_n*B)*B'*P_n*A;
    P_t(((n-1)*4+1):n*4,:) =P;
    P_n = P;
end

P_0 = P;
K_0 = -inv(R+B'*P_0*B)*B'*P_0*A;

U_lqr = K_0*X;

end