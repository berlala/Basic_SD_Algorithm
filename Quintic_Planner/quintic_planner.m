%% Quintic Plan Plan 
% Bolin Zhao , FF China, bolin.zhao@ff.com
%%
function [xt_log, vt_log, at_log, jt_log] = quintic_planner(xs, vxs,axs,xe,vxe,axe,T,t) 

A = [T^3, T^4, T^5;
         3*T^2, 4*T^3, 5*T^4;
         6*T, 12*T^2, 20*T^3];
     
b = [xe-xs-vxs*T-axs*T^2,
        vxe-vxs-2*axs*T,
        axe-2*axs];
   
x  = linsolve(A, b);

a0 = xs;
a1 = vxs;
a2 = axs;
a3 = x(1);
a4 = x(2);
a5 = x(3);

xt_log = [];
vt_log = [];
at_log =[];
jt_log =[];

for ts = 0:t:T
    xt = a0 + a1* ts + a2*ts^2 + a3*ts^3 + a4*ts^4 + a5*ts^5;
    xt_log = [xt_log, xt];
    
    vt = a1+ 2*a2*ts +3*a3*ts^2 + 4*a4*ts^3 + 5*a5*ts^4;
    vt_log = [vt_log, vt];
    
    at =  2*a2 + 6*a3*ts + 12*a4*ts^2 + 20*a5*ts^3;
    at_log = [at_log, at];

    jt =  6*a3 + 24*a4*ts + 60*a5*ts^2;
    jt_log = [jt_log, jt];

end
return