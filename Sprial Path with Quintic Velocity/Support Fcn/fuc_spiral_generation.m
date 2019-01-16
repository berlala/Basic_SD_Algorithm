%%%%%%%%%%%%%%%%%%%%%%% spiral generation %%%%%%%%%%%%%%%%%%%%%%
% for the generation of optimal spiral path (3 order)
% read the following paper in detail for reference
% 'Reactive Nonholonomic Trajectory Generation via Parametric Optimal Control'
%%%%%%%%%%%%%%%%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%
% Spiral_Msg_Path_Point = [ x y theta k s] in ego ISO coordinate
% x, y: coordinate;theta: heading;k: curvature;s: distance;
% Spiral_Msg_Q = [a b c d S], this q is the final q
%%%%%%%%%%%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%%
% point_start, point_end: the start point and end point;
% point = [x y theta k], point description; in global coordinate
% step_length: step distance between the two nearby waypoints;
% spiral_init_param_mat [phi theta k a b c d s], the intial parameters matrix of
% the the spiral
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/02/15, create;
% WILL, 2018/03/03, verified;
% WILL, 2018/03/16£¬ change the spiral_init_param_mat as an input from load;

function [Spiral_Msg_Q, Spiral_Msg_Path_Point] = fuc_spiral_generation( start_point, end_point, step_length, spiral_init_param_mat)

    % external function declaration
%     eml.extrinsic('plot');  

    % data type
    start_point = double(start_point);
    end_point = double(end_point);
    spiral_init_param_mat = double(spiral_init_param_mat);
    step_length = double(step_length);

    % define
    persistent CIRCLE_R;  
    if(isempty(CIRCLE_R))
        CIRCLE_R = 10;
    end
    
    % initial
    Spiral_Msg_Q = zeros(1,5);
    Spiral_Msg_Path_Point = single(zeros(1,5));

    % change the end_point position into ego(start_point) ISO  coordinate frame
    fit_point = [ fuc_gc_trasf_to_lcISO( end_point(:,1:3), start_point(1:3)) end_point(4)];
    fit_point(1,3) = fuc_regulate_angle(fit_point(1,3));
              
    % set a proprotion to make the distace fix in 10 meters
    % fix the fit_point in the first quadrant and conside the x-sysmmetry situation
    proprotion = CIRCLE_R / sqrt( power(fit_point(1,1),2) + power(fit_point(1,2),2) );
    if ( fit_point(1,2) >= 0 )
        sgn = 1;
    else
        sgn = -1;
    end
    fit_point = [ ( fit_point(1,1)*proprotion ) ...
                  ( fit_point(1,2)*proprotion*sgn ) ...
                  ( fit_point(1,3)*sgn ) ...
                  ( fit_point(1,4)*sgn/proprotion ) ];
            
    % find the init value of q [a b c d S], the parameter of the spiral expression
    % define the init lambda for iteration
    q_init_a = start_point(4)*sgn/proprotion;
    q_init_tail4 = fuc_find_init_param( atan2(fit_point(1,2),fit_point(1,1)), fit_point(1,3), q_init_a, spiral_init_param_mat );
    if isempty(q_init_tail4)
        % no inital parameters
        return;
    end
    q_iter_out = [q_init_a q_init_tail4]; % inital value
    lambda_iter_out = [1 1 1 1]; % inital value
    
    % iteration matrix update
    [grad_solve_mat_left, grad_solve_mat_right] = fuc_ite_matrix_update(q_iter_out, lambda_iter_out, fit_point);
    
    % outer iteration process
    path_end_point = fuc_gen_path_end(q_iter_out);
    error_out = fit_point - path_end_point;  % inital error
    error_out(3) = fuc_regulate_angle(error_out(3));
    iter_count_out = 0; % inital value
    while ( (error_out * error_out' > 0.001) && (iter_count_out < 100) )
        % solve [b,c,d,s,lambda1,lambda2,lambda3,lambda4]'
        delta_q_lambda = linsolve(grad_solve_mat_left,grad_solve_mat_right);
        if ( max(delta_q_lambda(2:end)) > 100 )
%             fprintf('delta_q_lambda'': %f  \n', delta_q_lambda);
%             fprintf('[Spiral Error] spiral cannot be fit delta \n');
            return;
        end
        
        % inner interation init
        error_in = [10 10 10 10];
        iter_count_in = 0; 
        q_iter_in = zeros(1,length(q_iter_out));
        lambda_iter_in = zeros(1,length(lambda_iter_out));
        step_size = 1;
        while ( ( error_in*error_in' > error_out*error_out' ) && ( iter_count_in < 10 ) )
            % update lambda and q
            lambda_iter_in = lambda_iter_out + step_size*delta_q_lambda(5:end)';
            q_iter_in = q_iter_out + step_size*[0 delta_q_lambda(1:4)'];
            if ( q_iter_in(end) <= 0 )
%                 fprintf('[Spiral Error] spiral length is less than zero (inner cycle)! \n');
                break;
            end
            
            % update inner error and step size
            path_end_point = fuc_gen_path_end(q_iter_in);
            error_in = fit_point - path_end_point;
            error_in(3) = fuc_regulate_angle(error_in(3));
            step_size = step_size * 0.5;
            
            % counter ++
            iter_count_in = iter_count_in + 1;
%             fprintf('iter_count_in: %f \n', iter_count_in);
        end
        
        % update outer lambda, q, error, and iteration matrix
        q_iter_out = q_iter_in;
        lambda_iter_out = lambda_iter_in;
        error_out = error_in;
        if ( q_iter_out(end) <= 0 )
%             fprintf('q_iter_out: %f \n', q_iter_out);
%             fprintf('[Spiral Error] spiral length is less than zero(outer cycle)! \n');
            break;
        end
        [grad_solve_mat_left, grad_solve_mat_right] = fuc_ite_matrix_update(q_iter_out, lambda_iter_out, fit_point);
        
        % counter ++
        iter_count_out = iter_count_out + 1;
%         fprintf('iter_count_out: %f \n', iter_count_out);
    end
       
    if( (error_out * error_out' > 0.001) || (q_iter_out(end) <= 0) )
%         fprintf('error_out: %f ', error_out);
%         fprintf('error_out'': %f  \n', error_out * error_out');
%         fprintf('[Spiral Error]  fail to fit spiral \n');
        return;
    end
        
    % generate the discreted path points
    temp = zeros(1,length(q_iter_out)-1);
    for i=1:1:length(temp)
        temp(i) = power(proprotion, i);
    end
    % final q and path points
    Spiral_Msg_Q = [q_iter_out(1:4).*temp*sgn   q_iter_out(end)/proprotion];
    Spiral_Msg_Path_Point = fuc_gen_path_points(Spiral_Msg_Q, step_length);
    
end