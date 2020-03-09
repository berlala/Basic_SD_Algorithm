
% INEQUALITY CONSTRAINTS
function [Ck, ug, lg] = getInequalityConstraints(border,MPC_vars,ModelParams)
    nx = ModelParams.nx;
    nu = ModelParams.nu;
    x1=border(1);
    y1=border(2);
    x2=border(3);
    y2=border(4);
    
    % numerator and denominator of slope of border. m = - (x2-x1)/(y2-y1)
    numer=-(x2-x1);
    denom=(y2-y1);

    dbmax=max(numer*x1-denom*y1,numer*x2-denom*y2);
    dbmin=min(numer*x1-denom*y1,numer*x2-denom*y2);

    Ck=zeros(1,nx+nu);
    Ck(1,1:2)=[numer -denom];
    ug= dbmax;
    lg= dbmin;
    
    Ck = Ck*blkdiag(MPC_vars.invTx,MPC_vars.invTu);
    
    
end