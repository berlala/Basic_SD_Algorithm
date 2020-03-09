%Compare Result
%%

figure(1)
Ts = 0.01;
plot((1:length(bl_LQR))*Ts, bl_LQR*180/pi);hold on
plot((1:length(guan_LQR))*Ts, guan_LQR*180/pi)
plot((1:length(bl_MPC))*Ts, bl_MPC*180/pi);
load mpc_change2
Ts = 0.05;
plot((1:length(delta_log))*Ts, delta_log*180/pi)
legend('Bolin LQR', 'Guan LQR', 'Bolin MPC')
ylabel('Front wheel SA[rad]')

figure(2)
Ts = 0.01;
plot((1:length(bl_LQR))*Ts, e_bl_lqr);hold on
plot((1:length(guan_LQR))*Ts, e_guan_lqr)
plot((1:length(e_mpc))*Ts, e_mpc);
Ts =0.05;
plot((1:length(e_log))*Ts, e_log);
legend('Bolin LQR', 'Guan LQR', 'Bolin MPC')
ylabel('m')

