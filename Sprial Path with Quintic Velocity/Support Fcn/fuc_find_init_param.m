%%%%%%%%%%%%%%%%%%%%%%% find init param %%%%%%%%%%%%%%%%%%%%%%
% find the init parameter of the spiral
%%%%%%%%%%%%%%%%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%
% 'Q_Init' is the parameter of the spiral expression
% [a b c d S],  k(s) = a + b*s + c*s^2 + d*s^3, 0 <= s <= S;
%%%%%%%%%%%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%%
% phi,theta,k, messages of the end_point in the start_point ISO coordinate 
% spiral_init_param_mat [phi theta k a b c d s], the intial parameters matrix of
% the the spiral
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/02/23, created, verified;
% WILL, 2018/03/16, change the spiral_init_param_mat as an input from load;
% WILL, 2018/09/13, add the limitation of phi, theta and k;

function  [ Q_Init_Tail4 ] = fuc_find_init_param( phi, theta, k, spiral_init_param_mat )


%     % the cols of the mat file is [ phi theta k a b c d S ]
      % phi      [0:pi/36:pi/2]
      % theta    [x:pi/36:x+pi]
      % k        [-0.25:0.05:0.25]
%     spiral_init_param_mat = load('threeD_spiral_param.mat');
    
    % find closest param in the inittable, the table has 19 sampled phi, 37 sampled theta, 11 sampled k
    Q_Init_Tail4 = [];
    if (phi<0)||(phi>0.5*pi)||(k<(-0.25))||(k>0.25)
        % add the limitation of phi and k
        return;
    end
    index_line = 1;
    index_line = index_line + min(fix(phi/(pi/180*5))*37*11, 18*37*11);
    theta_bund_low = spiral_init_param_mat(index_line, 2);
    if (theta<theta_bund_low)||((theta-theta_bund_low)> pi)
        % add the limitation of theta
        return;
    end
    index_line = index_line + fix( (theta - theta_bund_low)/(pi/36) ) * 11 + min( fix((k+0.25)/0.05) ,10);  % pi/180*5 = pi/36
    Q_Init_Tail4 = spiral_init_param_mat(index_line, 5:8);
end