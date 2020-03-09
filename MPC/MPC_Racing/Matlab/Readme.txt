Bolin:

Run simulation.m to conduct.

Change log:
2018.12.24
0) former optimizer function is change to optimizer_MPC due to confict with YALMIP lib;
1) ipopt solver within YALMIP is used;
2) In getMPC_vars(), remove the normalization. Use the general format instead;
3) prediction horizon is 20;
