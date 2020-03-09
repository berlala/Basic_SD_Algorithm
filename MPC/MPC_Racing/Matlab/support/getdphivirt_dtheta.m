
function dphivirt_dtheta=getdphivirt_dtheta(theta_virt,pathinfo)
    % computes {d phi_virt / d theta} evaluated at theta_k

    dxdth=ppval(pathinfo.dppx,theta_virt); %d x_virt / d theta
    dydth=ppval(pathinfo.dppy,theta_virt); %d y_virt / d theta
    d2xdth2=ppval(pathinfo.ddppx,theta_virt); %d2 x_virt / d theta2
    d2ydth2=ppval(pathinfo.ddppy,theta_virt); %d2 y_virt / d theta2

    numer=dxdth*d2ydth2 - dydth*d2xdth2;
    denom=dxdth^2 + dydth^2;

    dphivirt_dtheta=numer/denom;
end