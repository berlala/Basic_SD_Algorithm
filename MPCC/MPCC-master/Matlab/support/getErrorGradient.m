


function [grad_eC, grad_eL] = getErrorGradient(pathinfo, theta_virt, ModelParams, x_phys, y_phys)

    [deC_dtheta, deL_dtheta, cos_phi_virt, sin_phi_virt] = getderror_dtheta(pathinfo, theta_virt, x_phys, y_phys);
    
    grad_eC = [ sin_phi_virt, -cos_phi_virt, zeros(1, ModelParams.nx-3), deC_dtheta];
    grad_eL = [-cos_phi_virt, -sin_phi_virt, zeros(1, ModelParams.nx-3), deL_dtheta];
end