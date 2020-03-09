%% Route Builder
% bolin.zhao@ff.com
clear;
%%
load Route_key_points.mat;
load_spiral_init_para;
% [x,y,heading,curv]

%% Process Path Points
v_add = zeros(length(point_array),1);
Path_points_overall = [point_array,v_add];

Path_points_overall(:,4) = 0; % condition to generate path

%% 1) need to increase points
% patch_density = 100;
% Path_points_part = [];
% for i = 1:length(Path_points_overall)-1
% distance = sqrt((Path_points_overall(i+1,1)-Path_points_overall(i,1))^2 + (Path_points_overall(i+1,2)-Path_points_overall(i,2))^2 );
% patchs = floor(distance/patch_density);
% 
% x_patch  = linspace(Path_points_overall(i,1), Path_points_overall(i+1,1), patchs);
% y_patch  = linspace(Path_points_overall(i,2), Path_points_overall(i+1,2), patchs);
% h_patch  = linspace(Path_points_overall(i,3), Path_points_overall(i+1,3), patchs);
% c_patch  = linspace(Path_points_overall(i,4), Path_points_overall(i+1,4), patchs);
% v_patch  = linspace(Path_points_overall(i,4), Path_points_overall(i+1,4), patchs);
% clear  Path_tmp_parts
% Path_tmp_parts = [x_patch;y_patch;h_patch;c_patch;v_patch]';
% Path_points_part = [Path_points_part;Path_tmp_parts]; %#ok<AGROW>
% end
% Path_points  = Path_points_part; % end of process
%% 2) Do not need to increase points
 Path_points = Path_points_overall;

%% Plot all the Key Points on the final figure
figure(1)
plot(Path_points(:,1), Path_points(:,2),'o');hold on
%% Route Generation
%
step_length = 0.2;
v_max = 10/3.6;
RouteMapArray_gen = [];

t = 0.01;
path_output = [];
%
for ii = 1:length(Path_points(:,1))-1
if ii ==1    
start_point =Path_points(ii,:); % Initial Global [X, Y, Heading, Curvature, Speed]
else
    start_point = RouteMapArray_gen(end,1:5);
end

end_point = Path_points(ii+1,:); % Final Gloabl [X, Y, Heading, Curvature, Speed]

% figure(1)
% plot(start_point(1), start_point(2),'x')
% plot(end_point(1), end_point(2),'*')

if start_point(1:2) == end_point(1:2)
    % continue
else
 start_point(4) =0; %patch for successful generation 

[q, Spiral_Msg ] = fuc_spiral_generation(start_point, end_point, step_length,threeD_spiral_param);

v_final = zeros(1,length(Spiral_Msg(:,1)));
a_final = v_final;
clear path
path = [Spiral_Msg,v_final',a_final']; % x y theta k s v a

%% Convert from LOCAL to GB
RouteMapArray = double(path);
Local_XY = RouteMapArray(:,1:2);

Heading = (RouteMapArray(1,3)-start_point(3) )*180/pi;
RotMat = [cosd(Heading), sind(Heading); ...
                   -sind(Heading), cosd(Heading)];
Local_GB = [];
for i = 1:length(Local_XY)
    Local_GB(i,:) =   RotMat*[Local_XY(i,1), ...
                                                   Local_XY(i,2)]';
    Local_GB(i,:)  =  Local_GB(i,:) +    start_point(1:2);        
end
%Heading_GB = mod(RouteMapArray(:,3)+start_point(3),pi);
Heading_GB =RouteMapArray(:,3)+start_point(3);
if ~isempty(RouteMapArray_gen) 
RouteMapArray(:,5)  = RouteMapArray(:,5) + RouteMapArray_gen(end,5);
end
RouteMapArray_m   = [Local_GB,Heading_GB,RouteMapArray(:,4:end)] ;  

%%
RouteMapArray_gen = [RouteMapArray_gen;RouteMapArray_m];
clear  RouteMapArray_m
end
end % end  of the iteration of generation 
%%
figure(1)
plot(RouteMapArray_gen(:,1), RouteMapArray_gen(:,2))
title('Generated Route Map')
xlabel('X')
ylabel('Y')