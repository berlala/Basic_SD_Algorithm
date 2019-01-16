
%%%%%%%%%%%%%%%%%%%%%%% regulate angle %%%%%%%%%%%%%%%%%%%%%%
% make the angle keep between in [-pi, pi]
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/02/15, created, verified;

function [Theta] = fuc_regulate_angle(theta)

    while (theta > pi)
        theta = theta - pi * 2;
    end

    while (theta < (-pi))
        theta = theta + pi * 2;
    end
    
    Theta = theta;

end