
function [deC_dtheta, deL_dtheta, cos_phi_virt, sin_phi_virt] = getderror_dtheta(pathinfo, theta_virt, x_phys, y_phys)
    dxvirt_dtheta=ppval(pathinfo.dppx,theta_virt); %d x_virt / d theta
    dyvirt_dtheta=ppval(pathinfo.dppy,theta_virt); %d y_virt / d theta
    
    phi_virt=atan2(dyvirt_dtheta,dxvirt_dtheta); %orientation of virtual position
    % virtual positions
    x_virt=ppval(pathinfo.ppx,theta_virt);
    y_virt=ppval(pathinfo.ppy,theta_virt);
    
    % difference in position between virtual and physical
    Dx=x_phys-x_virt;
    Dy=y_phys-y_virt;

    dphivirt_dtheta=getdphivirt_dtheta(theta_virt,pathinfo);

    cos_phi_virt=cos(phi_virt);
    sin_phi_virt=sin(phi_virt);

    tmp1=[dphivirt_dtheta, 1];
    tmp2=[cos_phi_virt; sin_phi_virt];

    MC=[ Dx Dy; dyvirt_dtheta -dxvirt_dtheta];
    ML=[-Dy Dx; dxvirt_dtheta  dyvirt_dtheta];

    deC_dtheta = tmp1 * MC * tmp2;
    deL_dtheta = tmp1 * ML * tmp2;
end