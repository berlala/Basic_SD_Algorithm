%%%%%%%%%%%%%%%%%%%%%%% generate path end %%%%%%%%%%%%%%%%%%%%%%
% generate the end point of the path
% eq.37 and eq.38 of the paper
%%%%%%%%%%%%%%%%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%
% 'Path_End_Point' = [x y theta k]
%%%%%%%%%%%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%%
% 'q' = [a b c d S], the parameter of the spiral expression
% 'lambda' = [1 1 1 1] (init value), scaling and symmetry factor
% 'fit_point', the end_point position in ego(start_point) ISO  coordinate frame
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/02/23, created;
% WILL, 2018/02/24, verified;

function  [Path_End_Point] = fuc_gen_path_end(q)
    
    num_q = length(q);
    num_p = num_q - 1;
    p = q(1:num_p);
    
    % calculate the final value 
%     Path_End_Point = [ integral(@(s) fuc_cos(s,p,0),0,q(end)) ...
%                        integral(@(s) fuc_sin(s,p,0),0,q(end)) ...
%                        polyval( p(num_p:-1:1)./linspace(num_p, 1, num_p), q(end) )*q(end) ...
%                        polyval( p(num_p:-1:1), q(end) ) ];
    Path_End_Point = [ fuc_simp_intg('cos',0,q(end),p,0) ...
                       fuc_simp_intg('sin',0,q(end),p,0) ...
                       polyval( p(num_p:-1:1)./linspace(num_p, 1, num_p), q(end) )*q(end) ...
                       polyval( p(num_p:-1:1), q(end) ) ];

end
