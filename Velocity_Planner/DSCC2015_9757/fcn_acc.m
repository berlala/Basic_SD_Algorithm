function acc_max = fcn_acc(v)
m = 1800; % total mass
r = 0.25; % tire radius
T_acc_engine = 240-v*10;

acc_max = T_acc_engine*13/(m*r);

end