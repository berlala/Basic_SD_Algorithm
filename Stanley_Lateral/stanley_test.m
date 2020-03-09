% Stanley Test
%%
%Bolin ZHAO, bolinzhao@yahoo.com
%%
clear;close all
load path_2222;
Path_loc = route;
%%

preview = 2; % without Preview the 
x_c = Path_loc(1,1)+0.01;
y_c = Path_loc(1,2)-0.02;
psi_c = Path_loc(1,3);
v_0 = 4;
lf = 1.5;
%%
Ts = 0.05;

for n = 1:50

v = v_0;    
CurPos = [x_c,y_c,psi_c];
[index,dis_nearest,X_clst] = find_closest(Path_loc,CurPos);
RefPos = Path_loc(min(length(Path_loc),index+preview),:);
%delta_f = stanley_lat(RefPos,CurPos,v) ;
delta_f = stanley_lat2(Path_loc,CurPos,v) ;

%%%%REAL WORLD STATE UPDATE Simulation%%%
x_c   = x_c + v * cos(psi_c) * Ts; %global position
y_c   = y_c + v * sin(psi_c) * Ts;
psi_c = psi_c + v /(lf)*tan(delta_f) * Ts;  % global heading
%%%%%%%%%%%%%%%%%%%%%%%%
x_log(n) = x_c;
y_log(n) = y_c;
psi_log(n) = psi_c;

end

%%
figure(1)
plot(Path_loc(:,1), Path_loc(:,2),'--');hold on
plot(x_log, y_log);
legend('Desire Path','Real Location')
