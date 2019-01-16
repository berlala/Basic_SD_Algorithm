
%%%%%%%%%%%%%%%%%%%%%%% global coordinate transfer to local coordinate (ISO) %%%%%%%%%%%%%%%%%%%%%%
% transfer the coordinate of the path point in local ISO from the global
%%%%%%%%%%%%%%%%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%
% 'Lc_Target_Pos [x y theta]' is the target position in local ISO coordinate
%%%%%%%%%%%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%%
% gc_target_pos [x y theta] in global coordinate
% gc_ego_pos [ x y theta ] in global coordinate
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/03/09, create, verified;

function [Lc_Target_Pos] = fuc_gc_trasf_to_lcISO( gc_target_pos, gc_ego_pos)

    tra_mat = [cos(gc_ego_pos(3))   sin(gc_ego_pos(3))  0  ...
               -cos(gc_ego_pos(3))*gc_ego_pos(1)-sin(gc_ego_pos(3))*gc_ego_pos(2); % for x
               -sin(gc_ego_pos(3))  cos(gc_ego_pos(3))  0  ...
               sin(gc_ego_pos(3))*gc_ego_pos(1)-cos(gc_ego_pos(3))*gc_ego_pos(2); % for y
               0                    0                   1  ...
               -gc_ego_pos(3)]; % for theta
    lines_temp = length(gc_target_pos(:,1));
    path_temp = zeros(lines_temp,4);
    path_temp = tra_mat * [gc_target_pos, ones(lines_temp,1)]';
    Lc_Target_Pos = path_temp';
    
end