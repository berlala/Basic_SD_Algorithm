%% Afirie Field Attract/Reject Test
%% 
close all;clear
density = 0.2;
Grid_X = 0:density:10;
Grid_Y = 0:density:10;
Basic_Z = ones(length(Grid_X), length(Grid_Y));
%surf(Grid_X,Grid_Y, Basic_Z);
P0 = 20;
a = 20; % factor for Ref force
b =10; % factor for Attract force
Rej_limit = 150; % limit of the single reject force

% [5 10 5] will make no-solution
%%
Goal = [10,10];
Obs = [3,2;
            3,3;
             5,7; 
             5.3,6;
             6,6;
             2,4;
             3,8;
             4,7;
             8,9];
         for k1 = 1: length(Grid_X)
             for k2 =1:length(Grid_Y)
                 X_c = Grid_X(k1); % current x position
                 Y_c = Grid_Y(k2); % current y position
                 Y_rre = [];
                 Y_ata = [];
                 rre =[];       
                 rat = sqrt((Goal(1)-X_c)^2 +(Goal(2)-Y_c)^2); % current location attract force from Goal
                 Y_ata = b*rat; % This is the only one value              
                 for k3 = 1:length(Obs)
                     rre(k3) =    sqrt((Obs(k3,1)-X_c)^2 +(Obs(k3,2)-Y_c)^2);
                     %          Y_rre = (1/rre(k3) - 1/P0)*1/(rre(k3))^2 * rat^2;
                     %          Y_ata =  (1/rre(k3) - 1/P0)^2 *rat;
                     Y_rre(k3) = a*(1/rre(k3)  - 1/P0 ) *1/(rre(k3)^2);
                     if  isinf(Y_rre(k3))==1|| Y_rre(k3)>150
                         Y_rre = Rej_limit; % a limited value
                     end
                 end
                 Field_rre(k1,k2) = sum(Y_rre); % SUM_FIELD of the reject force at current location
                 Field_ata(k1,k2) =  Y_ata;
             end
         end

% figure(1)
% surf(Grid_X,Grid_Y,Field_rre)
% figure(2)
% surf(Grid_X,Grid_Y,Field_ata)

SUM_FIELD = Field_rre  + Field_ata; %total force field
%Plot_avg = mean(SUM_FIELD);

figure(3)
colormap jet
surfc(Grid_X,Grid_Y,SUM_FIELD',...
    'FaceColor','interp',...
   'EdgeColor','none',...
   'FaceLighting','gouraud') % ATTENTION: the match of the ray/col of the Matrix and the X/Y-coordinate
hold on

%% Find the path
kk = 100;
pos_init = [1,1];%index 
pos_x = 1; % current X position
pos_y =1; %current Y position
pos_xh = [];
pos_yh = [];
pos_x_next = 1;
pos_y_next =1;
Mat_loc = zeros(3,3); % the local field

for i= 1:kk
    
  for k1 =1:3 % the for loop for 
      pos_k1 = pos_x-2+k1;
      for k2 =1:3
          pos_k2 = pos_y-2+k2;
          if pos_k1==0 ||  pos_k2==0||pos_k1>length(SUM_FIELD)|| pos_k2>length(SUM_FIELD)
              Mat_loc(k1,k2) = inf;
          else
              Mat_loc(k1,k2) =SUM_FIELD(pos_k1,pos_k2); %pick up the local force
          end
      end
  end
         Mat_loc(2,2) = inf; %Remove the center node to avoid the standing-still
         

         [loc_x, loc_y] = find(Mat_loc ==min(min(Mat_loc))); %Find the local next step
         pos_x_next = pos_x+loc_x-2;
         pos_y_next = pos_y+loc_y-2;
         
%          for k6 =1:length(pos_xh) %prevent go back
%              if pos_x_next ==pos_xh(k6)&&pos_y_next ==pos_yh(k6)
%                  Mat_loc(loc_x,loc_y) = inf;
%              end
%          end     
         pos_x = pos_x_next;
         pos_y = pos_y_next;
         if pos_x == Goal(1)/0.2+1 && pos_y == Goal(2)/0.2+1 % at the final goal position
           disp('Reach Goal!')
           break
         else
          pos_xh = [pos_xh, pos_x]; % Record
          pos_yh = [pos_yh, pos_y]; 
         end
         clear Mat_loc
end

figure(4)
plot(pos_xh-1, pos_yh-1, 'r-o')  
hold on
plot(Goal(1)/0.2,Goal(2)/0.2,'bv')
hold on
plot(Obs(:,1)/0.2, Obs(:,2)/0.2, '*','MarkerSize',P0*0.8)
legend('Path', 'Goal','Obs', 'location','best')
title('Path')

for k5 =1:length(pos_xh)
figure(3)
plot3((pos_xh(k5)-1)*density, (pos_yh(k5)-1)*density, SUM_FIELD(pos_xh(k5), pos_yh(k5)),'o-r')
hold on
end
title('Field')
