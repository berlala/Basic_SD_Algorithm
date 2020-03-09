%% Error Check

Ts = time(1);
index = 0;
for T = [0.5/Ts, 1/Ts,  3/Ts, 5/Ts, 8/Ts]
 index = index+1;
Error(index) = sqrt((x_2(T)-x_log(T))^2 + (y_2(T) - y_log(T))^2);
E_phi(index) = phi_2(T) - psi_log(T)/pi*180;
end

Error
E_phi