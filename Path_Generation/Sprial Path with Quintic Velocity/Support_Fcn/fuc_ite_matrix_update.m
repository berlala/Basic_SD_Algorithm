%%%%%%%%%%%%%%%%%%%%%%% iteration matrix update %%%%%%%%%%%%%%%%%%%%%%
% update the iteration matrix
% read the following paper in detail for reference
% 'Reactive Nonholonomic Trajectory Generation via Parametric Optimal Control'
% appendix A of the paper
%%%%%%%%%%%%%%%%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%
% 'Grad_Solve_Mat_Left' and 'Grad_Solve_Mat_Right' is the matirx shown in the eq.30 of the paper
%%%%%%%%%%%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%%
% 'q' = [a b c d S], the parameter of the spiral expression
% 'lambda' = [1 1 1 1] (init value), scaling and symmetry factor
% 'fit_point', the end_point position in ego(start_point) ISO  coordinate frame
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/02/23, created;
% WILL, 2018/03/03, verified;

function  [Grad_Solve_Mat_Left, Grad_Solve_Mat_Right] = fuc_ite_matrix_update(q, lambda, fit_point)
    
    % define
    PARAM_SIZE = 4; % for 3-order spiral

    % calculate the error at the end point (final status)
    path_end_point = fuc_gen_path_end(q);
    error = fit_point - path_end_point;
    error(3) = fuc_regulate_angle(error(3));
    
    % calculate the intermediate parameters
    % eq.61 & eq.62
    s_final = q(end);
    theta_final = path_end_point(3);
    k_final = path_end_point(4);
%     k_final_dot = polyval( q(4:-1:2).*linspace(3, 1, 3), s_final );
    k_final_dot = q(2) + 2*q(3)*s_final + 3*q(4)*power(s_final,2);
    k_final_doub_dot = 2*q(3) + 6*q(4)*s_final;
    
    % eq.63 
    ptheta_pq = zeros(1,PARAM_SIZE+1);
    pk_pq = zeros(1, PARAM_SIZE+1);
    temp = zeros(1,PARAM_SIZE+1);
    for i=1:1:PARAM_SIZE
        ptheta_pq(i) = power(s_final, i) / i;
        pk_pq(i) = power(s_final, (i-1) );
        temp(i) = power(s_final, (i-2) ) * (i-1); 
    end
    ptheta_pq(PARAM_SIZE+1) = k_final;
    pk_pq(PARAM_SIZE+1) = k_final_dot;
    temp(PARAM_SIZE+1) = k_final_doub_dot;
    
    % eq.64 and eq.65
    p2theta_pq2 = zeros(PARAM_SIZE+1);
    p2theta_pq2(PARAM_SIZE+1,:) = pk_pq;
    p2theta_pq2(:,PARAM_SIZE+1) = pk_pq';
    p2k_pq2 = zeros(PARAM_SIZE+1);
    p2k_pq2(PARAM_SIZE+1,:) = temp;
    p2k_pq2(:,PARAM_SIZE+1) = temp';
    
    % eq.66, eq.67, eq.68
    sNs_val = zeros(1,PARAM_SIZE * 2 + 1);
    cNs_val = zeros(1,PARAM_SIZE * 2 + 1);
    for i=2:1:(PARAM_SIZE * 2 + 1)  
        sNs_val(i) = fuc_simp_intg('sin',0, s_final, q(1:4), i-1);
        cNs_val(i) = fuc_simp_intg('cos',0, s_final, q(1:4), i-1);
        
    end
    
    % eq.70, eq.71, eq.72
    px_pq = zeros(1,PARAM_SIZE+1);
    py_pq = zeros(1,PARAM_SIZE+1);
    p2x_pq2 = zeros(PARAM_SIZE+1);
    p2y_pq2 = zeros(PARAM_SIZE+1);
    for i=1:1:PARAM_SIZE
        px_pq(i) = - sNs_val(i+1) / i;
        py_pq(i) = cNs_val(i+1) / i;
        for j=1:1:PARAM_SIZE
            p2x_pq2(i, j) = - cNs_val(i + j + 1) / (i * j);
            p2y_pq2(i, j) = - sNs_val(i + j + 1) / (i * j);
        end
    end
    px_pq(PARAM_SIZE+1) = cos(theta_final);
    py_pq(PARAM_SIZE+1) = sin(theta_final);
    for j=1:1:PARAM_SIZE
        p2x_pq2(PARAM_SIZE+1, j) = - power(s_final, j) * sin(theta_final) / j;
        p2x_pq2(j, PARAM_SIZE+1) = p2x_pq2(PARAM_SIZE+1, j);
        p2y_pq2(PARAM_SIZE+1, j) = power(s_final, j) * cos(theta_final) / j;
        p2y_pq2(j, PARAM_SIZE+1) = p2y_pq2(PARAM_SIZE+1, j);
    end
    p2x_pq2(PARAM_SIZE+1, PARAM_SIZE+1) = - k_final * sin(theta_final);
    p2y_pq2(PARAM_SIZE+1, PARAM_SIZE+1) = - k_final * cos(theta_final);
    
    % eq.76, eq.77, eq.79
    pJk_pq = zeros(1,PARAM_SIZE+1);
    p2Jk_pq2 = zeros(PARAM_SIZE+1);
    for i =1:1:PARAM_SIZE
        pJk_pq(i) = polyval( q(PARAM_SIZE:-1:1)./linspace(i+PARAM_SIZE-1, i, PARAM_SIZE), s_final )*power(s_final, i);
        for j=1:1:PARAM_SIZE
            p2Jk_pq2(i,j) = power(s_final, i+j-1) / (i+j-1);
        end
    end
    pJk_pq(PARAM_SIZE+1) = power(k_final,2) / 2;
    for j=1:1:PARAM_SIZE
        p2Jk_pq2(PARAM_SIZE+1, j) = power(s_final, j-1) * k_final;
        p2Jk_pq2(j, PARAM_SIZE+1) = p2Jk_pq2(PARAM_SIZE+1, j);
    end
    p2Jk_pq2(PARAM_SIZE+1, PARAM_SIZE+1) = k_final * k_final_dot;
    
    % assemble the matrix as shown in eq.30, 
    pg_pq = [ px_pq; py_pq; ptheta_pq; pk_pq ];
    pH_pq = pJk_pq + lambda * pg_pq;  % eq.26
    lambda_p2g_pq2 = lambda(1) * p2x_pq2 + lambda(2) * p2y_pq2 +... 
                     lambda(3) * p2theta_pq2 + lambda(4) * p2k_pq2; % eq.29
    p2H_pq2 = p2Jk_pq2 + lambda_p2g_pq2; % eq.28
    Mat_Left = [ p2H_pq2 pg_pq'; pg_pq zeros(4)];
    Mat_Right = [ - pH_pq'; error'];
    Grad_Solve_Mat_Left = zeros(8);
    for i=1:1:8
        for j=1:1:8
            Grad_Solve_Mat_Left(i,j) = Mat_Left(i+1,j+1);
        end
    end
    Grad_Solve_Mat_Right = Mat_Right(2:end,1);
end
