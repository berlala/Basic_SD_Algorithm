%%%%%%%%%%%%%%%%%%%%%%%  sin/cos simpson integral functhion %%%%%%%%%%%%%%%%%%%%%%
% used for the calculation of simpson's ruler integration of s^n *
% sin(theta(p,s)) or cos(theta(p,s))
% read the following paper in detail for reference
% 'Reactive Nonholonomic Trajectory Generation via Parametric Optimal Control'
% appendix B of the paper
%%%%%%%%%%%%%%%%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%
% 'Simp_integ_result' integral result
%%%%%%%%%%%%%%%%%%%%%% input %%%%%%%%%%%%%%%%%%%%%%
% 'type', sin or cos
% 's_int, s_end' is the range of the integration
% 'p' [a b c d], for the description of theta(p,s)
% 'num', for the description of x(p,s) and y(p,s)
%%%%%%%%%%%%%%%%%%%%%% author %%%%%%%%%%%%%%%%%%%%%%
% WILL, 2018/03/12, created;
% WILL, 2018/03/12, verified;

function Simp_integ_result = fuc_simp_intg(type, s_init, s_end, p, num)

    % external function declaration

    % define
    samples = 30;
    num_p = length(p);
    
    w = ones(1,samples+1);
    for i=1:1:samples+1
        if (rem(i,2) == 0)
            w(i) = 2;
        else
            w(i) = 4;
        end
    end
    
    s_list = zeros(1,samples+1);
    s_list = fuc_linspace(s_init, s_end, samples+1);
    num_temp = zeros(1,num_p);
    num_temp = fuc_linspace(num_p, 1, num_p);
    theta = zeros(1,samples+1);
    theta =  fuc_polyval( p(num_p:-1:1)./num_temp, s_list);
    theta = theta.*s_list;
    
    switch type
        case 'sin'
            temp = power(s_list, num).*w.*sin(theta);
        case 'cos'
            temp = power(s_list, num).*w.*cos(theta);
        otherwise
            temp = 0;
    end
      
    Simp_integ_result = (s_list(2)-s_list(1))/3*sum(temp);

end