%% Quintic Plan Plan 
% Bolin Zhao , FF China, bolin.zhao@ff.com
%%
function [xt_log] = quintic_planner(xs, vxs,axs,xe,vxe,axe,T,t) 

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
for ts = 0:t:T
    xt = a0 + a1* ts + a2*ts^2 + a3*ts^3 + a4*ts^4 + a5*ts^5;
   xt_log = [xt_log, xt];
end
return