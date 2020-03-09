
% compute linear contouring and lag errors
function Qtilde = generateQtilde(pathinfo,MPC_vars,ModelParams,Xk,i)
    if i == MPC_vars.N+1
        Q = diag([MPC_vars.qCNmult*MPC_vars.qC, MPC_vars.qL]);
    else
        Q = diag([MPC_vars.qC, MPC_vars.qL]);
    end
        
    theta_virt=mod(Xk(end),pathinfo.ppx.breaks(end));
    [grad_eC, grad_eL] = getErrorGradient(pathinfo, theta_virt, ModelParams,Xk(1), Xk(2));
    errorgrad = [grad_eC; grad_eL]; 
    Qtilde = errorgrad'*Q*errorgrad; 
end