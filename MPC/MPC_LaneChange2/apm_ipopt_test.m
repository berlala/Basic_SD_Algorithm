clear all; close all; clc
disp('APM MATLAB available for download at http://apmonitor.com')
addpath('apm')
% 
% %% example Quadratic program
H = [1 -1; -1 2]; 
f = [-2; -6];
A = [1 1; -1 2; 2 1];
b = [2; 2; 3];
Aeq = [];
beq = [];
lb = zeros(2,1);
ub = [];
x0 = [];
% 
%% generate APMonitor QP model
y1 = apm_quadprog(H,f,A,b,Aeq,beq,lb,ub,x0)

%% compare solution to quadprog (MATLAB)
y2 = quadprog(H,f,A,b,Aeq,beq,lb,x0)
