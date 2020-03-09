function [i_target_pos,  X_state]= find_distance(horizonx, ind_clst,initial_ENH,RouteMapArray)

i_target_pos = zeros(length(horizonx),1);
[~,k_col] = size(RouteMapArray);
X_state = zeros(length(horizonx), k_col);
dis_t= 0;
x_c = initial_ENH(1);
y_c = initial_ENH(2);


for n = 1:length(horizonx)
    
    for ii = 1:(length(RouteMapArray(:,1)))
        index = ii*1+ind_clst(1); %drct
        % In D Gear: index increase from ind_clst to the last point;
        % In R Gear: index decrease from ind_clst to the inital point.
        max_length= length(RouteMapArray(:,1));
        if index > max_length
            index = max_length;
        end
        if index < 1
            index = 1;
        end
        x_t = RouteMapArray(index,1);
        y_t = RouteMapArray(index,2);
        dis_t = sqrt((x_t - x_c)^2 + (y_t - y_c)^2);
        if dis_t > horizonx(n)
            i_target_pos(n) = index; % The first Target
            X_state(n,:) = RouteMapArray(i_target_pos(n),:);
            break
        else
            i_target_pos(n) = length(RouteMapArray(:,1));
            X_state(n,:) = RouteMapArray(i_target_pos(n),:);
        end% if it apporach the last point
    end

end
    
end
