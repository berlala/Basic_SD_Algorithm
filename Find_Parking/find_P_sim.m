% FF China, IAI, Bolin ZHAO (bolin.zhao@ff.com)
% 2019/3/28
%Intro: this function is for finding the Parking space and location from a
%defined space.

%To Be Update: line164,line201, should not be [block] but the [rectangle].
%%
%input:
% Four point location: [a,b;c,d;e,f;g,h];

%output:
% Point_Target = [x;y;yaw]
% Type : 1 for Vertical, 3 for parallel
% size: [short edge, long edge] for final cube space.
%%
close all;clear;
Park_place_data;
block = GG;
situation = 0; % 1 for Trapezoid

%plot
figure(1)
plot(0,0 ,'*');hold on
for i = 1:4
   ii = i+1;
   if ii ==5
       ii =1;
   end
plot(block(i,1),block(i,2),'o');hold on
plot([block(i,1), block(ii,1)], [block(i,2), block(ii,2)],'linewidth',2);
end
%
a = block(1,:);
b = block(2,:);
c = block(3,:);
d = block(4,:);

dist = [];
for i = 1:4
    if i < 4
     ii = i+1;
    else
      ii = 1;
    end
    dist(i,1) = sqrt((block(i,1) - block(ii,1))^2 + (block(i,2) - block(ii,2))^2);
    dist(i,2) = int8(i);
end

[~, index] = sort(dist(:,1)); % increasing rank


%%  Judge the Type and Direction
% when the first edge is 4 or 3, means the first edge is long edge.
if (index(1) == 2||index(1) ==4) &&(index(3)==1 || index(3) == 3) % do not take xiefang into account
    Type = 3; %parallel
else
    Type = 1; %vertical
end
%%  Special Case 
if Type == 3  % parallel, two long 
    L11 = a;
    L12 = b;
    L21 = d;
    L22 = c;
elseif Type == 1 %vertical, two long
    L11 = a;
    L12 =d;
    L21 = b;
    L22 = c;
end

if Type ==3  % parallel case
    if L11(2) ==L12(2)
        Line1 = L11(2); % parallel to x-axis
    end
    if L21(2) == L22(2)
        Line2  = L21(2);
    end
end
 
if Type ==1 % vertical case
    if L11(1) ==L12(1)
        Line1 = L11(1); % parallel to y-axis
    end
    if L21(1) == L22(1)
        Line2  = L21(1);
    end
end

% General Case
if Type == 3 %affect the block(1,:) and block(2,:)
    block_v = zeros(4,2);
    for i = 3:4
        block_v(i,:) = fcn_3points(block(i,:),block(1,:), block(2,:));
    end
    for i = 1:2
        block_v(i,:) = fcn_3points(block(i,:),block(3,:), block(4,:));
    end
end
% [block_v] give the vertical point from each point under the sequence 1234 

if Type == 1   %Vertical Case
    block_v = zeros(4,2);
    for i = 2:3
        block_v(i,:) = fcn_3points(block(i,:),block(1,:), block(4,:));
    end
    for i = 1:3:4
        block_v(i,:) = fcn_3points(block(i,:),block(2,:), block(3,:));
    end
end
%%  Situation A: Trapezoid

%plote
for i = 1:4
plot(block_v(i,1),block_v(i,2),'x');hold on
plot([block_v(i,1), block(i,1)], [block_v(i,2), block(i,2)],'--');
end
    
if Type ==3
    point_set_1 =     [block(1,:);block_v(3,:) ;block_v(4,:) ;block(2,:)];
    point_set_2 =     [block(3,:); block_v(1,:) ; block_v(2,:);block(4,:)];
elseif Type ==1
    point_set_1 =     [block(2,:);block_v(1,:) ;block_v(4,:) ;block(3,:)];
    point_set_2 =     [block(1,:); block_v(2,:) ; block_v(3,:);block(4,:)];
end
    
%% Judge the Shape/Situation
% judge whether the two points in the middle, for Line1
Trag_1_1 = []; Trag_2_1 =[];

dis_all = []; dist_left = []; dist_right = [];
dis_all = fcn_dist(point_set_1(1,:), point_set_1(4,:));
for i  = 1:2%left point to two middle point
    dist_left(i) = fcn_dist(point_set_1(2,:), point_set_1(i*i,:));
end
for i  = 1:2%right point to two middle point
    dist_right(i) = fcn_dist(point_set_1(3,:), point_set_1(i*i,:));
end

%
if min(dist_left(:) <dis_all) && min(dist_right(:) <dis_all) % the two point in the middle of point1 and 2

    Trapezoid = [point_set_1(2:3,:); point_set_2(4,:); point_set_2(1,:)]; % the four point for the Trapezoid shape
    situation = 1;
elseif sum(dist_left(:) <dis_all)==2&&sum(dist_right(:) <dis_all)==1 %judge which point on the line 
    Trag_1_1 = [point_set_1(2,:);];
    if dist_left(1)  > dist_left(2)
        Trag_1_1 = [Trag_1_1;point_set_1(1,:)];
        situation  =2;
    else
        Trag_1_1 = [Trag_1_1;point_set_1(4,:)];
        situation  =2;
    end
elseif sum(dist_left(:) <dis_all)==1&&sum(dist_right(:) <dis_all)==2
    Trag_1_1 = [point_set_1(3,:);];
    if dist_right(1)  > dist_right(2)
        Trag_1_1 = [Trag_1_1;point_set_1(1,:)];
        situation  =2;
    else
        Trag_1_1 = [Trag_1_1;point_set_1(4,:)];  
          situation  =2;
    end
end
 
if (min(dist_left)== min(dist_right)) && (max(dist_left) == max(dist_right))%for line1 is [x,0]or[0,x]
    Trapezoid = block;
end


% for Line2
dis_all = []; dist_left = []; dist_right = [];
dis_all = fcn_dist(point_set_2(1,:), point_set_2(4,:));
for i  = 1:2%left point to two middle point
    dist_left(i) = fcn_dist(point_set_2(2,:), point_set_2(i*i,:));
end
for i  = 1:2%right point to two middle point
    dist_right(i) = fcn_dist(point_set_2(3,:), point_set_2(i*i,:));
end

if min(dist_left(:) <dis_all) && min(dist_right(:) <dis_all) % the two point in the middle of point1 and 2

        Trapezoid = [point_set_2(2:3,:);point_set_1(4,:);point_set_1(1,:)]; % the four point for the Trapezoid shape
    situation = 1;
elseif sum(dist_left(:) <dis_all)==2&&sum(dist_right(:) <dis_all)==1 %judge which point on the line 
    Trag_2_1 = [point_set_2(2,:);];
    if dist_left(1)  > dist_left(2)
        Trag_2_1 = [Trag_2_1;point_set_2(1,:)];
    else
        Trag_2_1 = [Trag_2_1;point_set_2(4,:)];
    end
elseif sum(dist_left(:) <dis_all)==1&&sum(dist_right(:) <dis_all)==2
    Trag_2_1 = [point_set_2(3,:);];
    if dist_right(1)  > dist_right(2)
        Trag_2_1 = [Trag_2_1;point_set_2(1,:)];
        situation  =2;
    else
        Trag_2_1 = [Trag_2_1;point_set_2(4,:)];  
          situation  =2;
    end
end
 
if (min(dist_left)== min(dist_right)) && (max(dist_left) == max(dist_right)) %for line2 is [x,0]or[0,x]
    Trapezoid = block;
end


% Build two Triangles
if situation ==2 
   if ~isempty(Trag_1_1)    
       target_point = Trag_1_1(2,:);
       %target_line  is Line2
       cross_1  = fcn_4points(target_point,a, b, c,d);
       Trag_1_1 = [Trag_1_1;cross_1];
   end
   if ~isempty(Trag_2_1)    
       target_point = Trag_2_1(2,:);
       %target_line  is Line2
       cross_2  = fcn_4points(target_point,c,d,a,b);
       Trag_2_1 = [Trag_2_1;cross_2];
   end
figure(1)
for i = 1:3
    if ~isempty(Trag_1_1)
        plot(Trag_1_1(i,1), Trag_1_1(i,2),'r*', 'linewidth',2)
    end
    if ~isempty(Trag_2_1)
        plot(Trag_2_1(i,1), Trag_2_1(i,2),'g*', 'linewidth',2)
    end
end

end
%% Base on the Trags to build the Trapezoids
%1) based on the Triangle to find the final point to consist rectangle;
%2) judge whether the final point is in the AREA
%3) if it is in, build the Trapezoids;
%4) if both exist (two trapezoids), judge the size
if situation ==2 
    % for first
    if ~isempty(Trag_1_1)
    [last_point] = fcn_rectangle(Trag_1_1(1,:), Trag_1_1(2,:),Trag_1_1(3,:)); % step1,  get the last point
    plot(last_point(1), last_point(2),'y*','linewidth',5)  
    [in_shape_1] = fcn_inshape(last_point, block); % step2 , check whether the  last point in SHAPE
    if in_shape_1 == 1
        Trapezoid = [Trag_1_1;last_point];
    end
    end
     % for second  
    if ~isempty(Trag_2_1) 
    [last_point] = fcn_rectangle(Trag_2_1(1,:), Trag_2_1(2,:),Trag_2_1(3,:)); % step1,  get the last point
    plot(last_point(1), last_point(2),'b*','linewidth',5)  
    [in_shape_2] = fcn_inshape(last_point, block); % step2 , check whether the  last point in SHAPE
    if in_shape_2 == 1
        Trapezoid = [Trag_2_1;last_point];
    end   
         end
end
%% Process the Trapezoid
%plot
for i = 1:4
    if i ==4
        ii =1;
    else
        ii =i+1;
    end
    plot(Trapezoid(i,1),Trapezoid(i,2),'x');hold on
    plot([Trapezoid(i,1), Trapezoid(ii,1)], [Trapezoid(i,2), Trapezoid(ii,2)],'--');
end

dist_final = [];
for i = 1:4
    if i ==4
        ii =1;
    else
        ii =i+1;
    end
   dist_final(i)  = fcn_dist(Trapezoid(i,:), Trapezoid(ii,:));
   dist_points(i,:) = [Trapezoid(i,:),Trapezoid(ii,:)];
end
 [~, index_d] =sort(dist_final); 
 
%  size  = [dist_final(index_d(1)), dist_final(index_d(2))];
%  Point_P = [((dist_points(index_d(1),1))+dist_points(index_d(2),3))/2,...
%                   ((dist_points(index_d(1),2))+dist_points(index_d(2),4))/2]
triangle_1  = [dist_points(index_d(1),1), dist_points(index_d(1),2)];
triangle_2  = [dist_points(index_d(1),3), dist_points(index_d(1),4)];
find = 0;
for i = 2:4
    if (index_d(i) == index_d(1)+1 || index_d(i) ==index_d(1)-1)&&find == 0
        index_next = index_d(i);
        find  = 1;
    end
end
%[index_next] 
triangle_3  = [dist_points((index_next),1), dist_points((index_next),2)];
triangle_4  = [dist_points((index_next),3), dist_points((index_next),4)];

size = [dist_final(index_d(1)), dist_final((index_next))];

%%
points = [triangle_1;triangle_2;triangle_3;triangle_4];

compare_index = [];iii=1;
for i = 1:4
    a = points(i,:);
    cum = 0;
    for ii = 1:4
        b = points(ii,:);
        cum = cum+double(min(a==b));
    end
    if cum == 1
    compare_index(iii) =i; % triangle long edge
    iii=iii+1;
    end
end

Point_P = [(points(compare_index(1),1)+points(compare_index(2),1))/2;
    (points(compare_index(1),2)+points(compare_index(2),2))/2]; % Car final location
% Final Yaw Angle 
Point_h = [(dist_points(index_d(1),1)+dist_points(index_d(1),3))/2,...
                (dist_points(index_d(1),2)+dist_points(index_d(1),4))/2]; % heading target point
            
            if Point_h(1) == Point_P(1)
                Yaw = pi/2*-1*sign(Point_P(2));
            elseif Point_h(2) == Point_P(2)
                Yaw = 0;
            else
                Yaw = atan2((Point_h(2) - Point_P(2)), (Point_h(1) - Point_P(1)));
                if Yaw < -pi/2&& Type  ==3 
                    Yaw = Yaw+pi;
                elseif Yaw >pi/2&& Type  ==3 
                    Yaw = Yaw-pi;
                elseif Yaw > 0 && Type == 1 && sign(Point_P(2)) ==1
                    Yaw = Yaw - pi;
                elseif Yaw < 0 && Type == 1 && sign(Point_P(2)) ==-1
                    Yaw = Yaw+pi;    
                end
            end

Point_Target = [Point_P;Yaw];            

 figure(1)
 plot(Point_P(1), Point_P(2), 'co','linewidth',2);hold on
 plot(Point_h(1), Point_h(2), 'co','linewidth',2);
 axis([-10,10,-10,10])
 %% Final report
 disp(Type)
 disp(size); %size for the final park place
 disp(situation) %situation for 1) 2)
 disp(Point_Target)

