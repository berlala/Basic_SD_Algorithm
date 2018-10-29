%% Quintic Plan Plan 
% Bolin Zhao , FF China, bolin.zhao@ff.com
%%
function [xt_log, vt_log, at_log, jt_log] = quintic_planner(xs, vxs,axs,xe,vxe,axe,T,t) 

A = [T^3, T^4, T^5;
         3*T^2, 4*T^3, 5*T^4;
         6*T, 12*T^2, 20*T^3];
     
B = [xe-xs-vxs*T-axs*T^2,
        vxe-vxs-2*axs*T,
        axe-2*axs];
   
x  = linsolve(A, B); % A*X =B, To solve all the coefficient

k0 = xs;
k1 = vxs;
k2 = axs;
k3 = x(1);
k4 = x(2);
k5 = x(3);

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