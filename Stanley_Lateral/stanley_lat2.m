function comAng_Stanley = stanley_lat2(RouteMapArray,initial_ENH,curvelocity)
%% Stanley Controller
k_h  =0.2; % this part is the key for stanley, it can only be the length of the wheelbase
horizonx = k_h*curvelocity;
Gain_f =1;
[ind_clst ,dis_nearest,X_clst]  = find_closest(RouteMapArray, initial_ENH);
[i_target_pos,  ~]= find_distance(horizonx, ind_clst, initial_ENH,RouteMapArray);

% if initial_ENH(3)>pi
%     psi_init = initial_ENH(3)- fix(initial_ENH(3)/(2*pi)) -2*pi;
% elseif initial_ENH(3)<-pi
%     psi_init = initial_ENH(3) +fix(initial_ENH(3)/(2*pi)) +2*pi;
% else
%     psi_init = initial_ENH(3) ;
% end

Heading = ( initial_ENH(3) )*180/pi; %[deg]
RotMat = [cosd(Heading), sind(Heading); ...
               -sind(Heading), cosd(Heading)];

pt0lc = RotMat*[X_clst(1)-initial_ENH(1),X_clst(2)-initial_ENH(2)]';
pt1lc = RotMat*[RouteMapArray(i_target_pos(1),1)-initial_ENH(1),RouteMapArray(i_target_pos(1),2)-initial_ENH(2)]';

sign_stan = 0;
if sign(pt0lc(2))== sign(pt1lc(2))==1 %same side
    sign_stan = double(sign(pt1lc(2))); 
elseif (sign(pt0lc(2))== sign(pt1lc(2)))==0
    sign_stan = double(sign(pt0lc(2)));
end

% revise the heading
% if RouteMapArray(i_target_pos,3)>pi
%     psi_tgt = RouteMapArray(i_target_pos,3) - fix(RouteMapArray(i_target_pos,3)/(2*pi)) -2*pi;
% elseif RouteMapArray(i_target_pos,3)<-pi
%     psi_tgt = RouteMapArray(i_target_pos,3) +fix(RouteMapArray(i_target_pos,3)/(2*pi)) +2*pi;
% else
%     psi_tgt = RouteMapArray(i_target_pos,3);
% end
  
psi_init = initial_ENH(3) ;
psi_tgt = RouteMapArray(i_target_pos,3);
theta_error =psi_tgt- psi_init;


df =sign_stan*dis_nearest; % Far away, FF+FB
comAng_Stanley = theta_error +  atan(Gain_f*df/horizonx); %[rad]


delta = sign(comAng_Stanley) * min(abs(comAng_Stanley), 35/180*pi);
end