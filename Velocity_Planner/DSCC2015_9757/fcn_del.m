function del_max = fcn_del(v)
m = 1800; % total mass
r = 0.25; % tire radius
g = 9.8;

F_brake = 0.8*m*g;

del_max = F_brake/m;

end