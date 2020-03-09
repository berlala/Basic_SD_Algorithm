%% Quintic Plan Plan 
% Bolin Zhao , FF China, bolin.zhao@ff.com
%%
function [xt_log, vt_log, at_log, jt_log] = quintic_planner(xs, vxs,axs,xg,vxg,axg,T,t) 

A = [0,0, 0,0,0,1;
         0,0,0,0,1,0;
         0,0,0,2,0,0; 
         T^5, T^4, T^3, T^2, T, 1;
         5*T^4, 4*T^3, 3*T^2, 2*T, 1, 0;
         20*T^3, 12*T^2, 6*T, 2, 0,0];
        
     
B = [xs,vxs,axs,xg,vxg, axg]';
  
x  = linsolve(A, B); % A*[k0~k5]'=B, To solve all the coefficient

k0 = x(6);
k1 = x(5);
k2 = x(4);
k3 = x(3);
k4 = x(2);
k5 = x(1);

xt_log = [];
vt_log = [];
at_log =[];
jt_log =[];

for ts = 0:t:T % The final result, functions based on sample time.
    xt = k0 + k1* ts + k2*ts^2 + k3*ts^3 + k4*ts^4 + k5*ts^5;
    xt_log = [xt_log, xt];
    
    vt = k1+ 2*k2*ts +3*k3*ts^2 + 4*k4*ts^3 + 5*k5*ts^4;
    vt_log = [vt_log, vt];
    
    at =  2*k2 + 6*k3*ts + 12*k4*ts^2 + 20*k5*ts^3;
    at_log = [at_log, at];

    jt =  6*k3 + 24*k4*ts + 60*k5*ts^2;
    jt_log = [jt_log, jt];
end
return