%% Track Builder
%%
clc;

sample = 0.1; %[m]
%% Part 1
s_x = 0;
s_y = 0;

straight  = 30;
Line_1_x  = s_x + 0:sample:straight;
Line_1_y  = s_y +zeros(1,length(0:sample:straight)); 
Line_1_K = zeros(1,length(0:sample:straight));

s_x = Line_1_x(end);
s_y = Line_1_y(end);
%% Part 2

r = 8;
x_1_c  = 30; 
y_1_c  = 8;
i = 1;
for x = 0:sample:8
   x_log(i) = x+x_1_c;
   y_log(i) = -sqrt(r^2 - (x_log(i) -x_1_c)^2) +y_1_c; % the negtive cannot be ignored
    i = i+1;
end
    
Line_2_x = x_log;
Line_2_y = y_log;
Line_2_K = 1/r*ones(1,length(x_log));

s_x = Line_2_x(end);
s_y = Line_2_y(end);
%% Part 3

straight  = 20;
Line_3_x  = s_x +zeros(1,length(0:sample:straight)); 
y_gain = 0:sample:straight;
Line_3_y  = s_y +y_gain ;  % why cannot use [s_y+0:sample:straight]???
Line_3_K = zeros(1,length(0:sample:straight));

s_x = Line_3_x(end);
s_y = Line_3_y(end);
%% Part 4:
r = 8;
x_2_c  = 30; 
y_2_c  = 28;
i = 1;
x_log = [];
y_log = [];
for x = 0:sample:r
   x_log(i) = x+x_2_c;
   y_log(i) = sqrt(r^2 - (x_log(i) -x_2_c)^2) +y_2_c; % the negtive cannot be ignored
    i = i+1;
end

    
    
Line_4_x = fliplr(x_log);
Line_4_y = fliplr(y_log);
Line_4_K = 1/r*ones(1,length(x_log));

s_x = Line_4_x(end);
s_y = Line_4_y(end);
%% Part 5

straight  = 10;

x_gain =0:-sample:-straight;
Line_5_x  = s_x + x_gain;
Line_5_y  = s_y +zeros(1,length(0:sample:straight)); 
Line_5_K = zeros(1,length(0:sample:straight));

s_x = Line_5_x(end);
s_y = Line_5_y(end);
%% Part 6
r = 8;
x_3_c  = s_x; 
y_3_c  = s_y+8;
i = 1;
x_log = [];
y_log = [];
for x = 0:sample:r
   x_log(i) = x_3_c-x;
   y_log(i) = -sqrt(r^2 - (x_log(i) -x_3_c)^2) +y_3_c; 
    i = i+1;
end
    
    
Line_6_x = x_log;
Line_6_y = y_log;
Line_6_K = -1/r*ones(1,length(x_log)); % turn right


s_x = Line_6_x(end);
s_y = Line_6_y(end);

%% Part 7
r = 8;
x_4_c  = s_x-8; 
y_4_c  = s_y;
i = 1;
x_log = [];
y_log = [];
for x = 0:sample:r
   x_log(i) = x;
   y_log(i) = sqrt(r^2 - (x_log(i) - 0)^2) ; 
    i = i+1;
end
    
Line_7_x = fliplr(x_log)+x_4_c ;
Line_7_y = fliplr(y_log) + y_4_c;
Line_7_K = 1/r*ones(1,length(x_log)); 

s_x = Line_7_x(end);
s_y = Line_7_y(end);
%% Part 8

straight  = 4;

x_gain =0:-sample:-straight;
Line_8_x  = s_x + x_gain;
Line_8_y  = s_y +zeros(1,length(0:sample:straight)); 
Line_8_K = zeros(1,length(0:sample:straight));

s_x = Line_8_x(end);
s_y = Line_8_y(end);

%% Part 9
r = 8;
x_5_c  = s_x; 
y_5_c  = s_y-8;
i = 1;
x_log = [];
y_log = [];
for x = 0:sample:r
   x_log(i) = -x;
   y_log(i) = sqrt(r^2 - (x_log(i) - 0)^2) ; 
    i = i+1;
end
    
Line_9_x = x_log+x_5_c ;
Line_9_y = y_log + y_5_c;
Line_9_K = 1/r*ones(1,length(x_log)); 

s_x = Line_9_x(end);
s_y = Line_9_y(end);

%% Part 10

straight  = 36;
Line_10_x  = s_x +zeros(1,length(0:sample:straight)); 
y_gain = 0:sample:straight;
Line_10_y  = s_y - y_gain ;  % why cannot use [s_y+0:sample:straight]???
Line_10_K = zeros(1,length(0:sample:straight));



s_x = Line_10_x(end);
s_y = Line_10_y(end);
%% Part 11

r = 8;
x_6_c  = s_x+8; 
y_6_c  = s_y;
i = 1;
x_log = [];
y_log = [];
for x = 0:sample:r
   x_log(i) = -x;
   y_log(i) = -sqrt(r^2 - (x_log(i) - 0)^2) ; 
    i = i+1;
end
    
Line_11_x = fliplr(x_log)+x_6_c ;
Line_11_y = fliplr(y_log) + y_6_c;
Line_11_K = 1/r*ones(1,length(x_log)); 

s_x = Line_11_x(end);
s_y = Line_11_y(end);


%% Add Lane
Line_x  = [Line_1_x, Line_2_x, Line_3_x, Line_4_x, Line_5_x, Line_6_x, Line_7_x, Line_8_x, Line_9_x, Line_10_x, Line_11_x]';
Line_y = [Line_1_y,Line_2_y, Line_3_y, Line_4_y, Line_5_y, Line_6_y , Line_7_y, Line_8_y, Line_9_y,Line_10_y, Line_11_y]';
Line_K = [Line_1_K,Line_2_K, Line_3_K, Line_4_K, Line_5_K, Line_6_K , Line_7_K, Line_8_K, Line_9_K,Line_10_K, Line_11_K]';


%% Overall Plot
 save track_bl.mat Line_x Line_y Line_K
plot(Line_x,Line_y)
title('Map')