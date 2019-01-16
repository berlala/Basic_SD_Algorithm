%%%%%%%%%%%%%%%%%%%%%%% generate path points %%%%%%%%%%%%%%%%%%%%%%
% generate the whole discreted point of the path
% read the following paper in detail for reference
% 'Reactive Nonholonomic Trajectory Generation via Parametric Optimal Control'
%%%%%%%%%%%%%%%%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%
% 'Path_Points' = [x' y' theta' k' s'] in ego(start_point) ISO  coordinate frame
%%%%%%%%%%%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%%
% 'q' = [a b c d S], the final parameter of the spiral expression
% 'step_lenth' the distance between two nearby path points
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/02/24, created, verified;

function [ Path_Points ] = fuc_gen_path_points(q, step_length)
    % external function declaration
    eml.extrinsic('fprintf');
    
    % define
    UP_BOUND_SIZE = 500;

    num_q = length(q);
    p = q(1:num_q-1);
    % every matrix need to define the up bound size
    num_points = min(ceil(q(end)/step_length)+1,UP_BOUND_SIZE);
%     fprintf('num_points: %f \n',num_points);
    
%     init
    s_mat = single(zeros(1,num_points));
    theta_mat = single(zeros(1,num_points));
    k_mat = single(zeros(1,num_points));
    x_mat = single(zeros(1,num_points));
    y_mat = single(zeros(1,num_points));
    
    % eq.37, eq.38, or eq.92
%     s_mat = linspace(0, q(end),num_points);
    s_mat_step = single(q(end))/single(num_points-1);
    for i=1:1:num_points
        s_mat(1,i) = s_mat_step*(single(i)-1);
        theta_mat(1,i) = single(fuc_polyval( q((num_q-1):-1:1)./[4 3 2 1], s_mat(1,i) ));
        k_mat(1,i) = single(fuc_polyval( q((num_q-1):-1:1), s_mat(1,i) )); 
        if i > 1
            x_mat(1,i) = x_mat(1,i-1) + single(fuc_simp_intg('cos',s_mat(1,i-1),s_mat(1,i),p,0));
            y_mat(1,i) = y_mat(1,i-1) + single(fuc_simp_intg('sin',s_mat(1,i-1),s_mat(1,i),p,0));
        end
    end
    theta_mat = theta_mat.* s_mat;
    
    % final assemble the path points matrix
    Path_Points = [x_mat; y_mat; theta_mat; k_mat; s_mat]';

end