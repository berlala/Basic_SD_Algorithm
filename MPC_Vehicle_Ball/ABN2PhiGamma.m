function [ Phi, Gamma ] = ABN2PhiGamma( A,B,N)
%ABN2 Summary of this function goes here
%   Detailed explanation goes here

nx = size(B,1);
nu = size(B,2);

% Phi
Phi = A;
if N>1
    for j = 2:N
        Phi     = [Phi;A^j];
    end
end

% Gamma
Gamma = zeros(nx*N,nu*N);
for i = 1:N
    for j = 1:i
        Gamma((i-1)*nx+1:nx*i,(j-1)*nu+1:j*nu) = A^(i-j)*B;
    end
end
end

