function xdot = f16_dynamics(x,u,xcg)
    % xcg: fraction of MAC

    xdot = zeros(size(x));
    
    f16_properties
    r2d = 57.29578;
    
    % unpack control variables
    
    thtl = u(1);
    elev = u(2);
    ail = u(3);
    rdr = u(4);
    
    % state variables
    Vt = x(1);
    alpha = x(2)*r2d;
    beta = x(3)*r2d;
    phi = x(4);
    theta = x(5);
    psi = x(6);
    p = x(7);
    q = x(8);
    r = x(9);
    alt = x(12);
    pow = x(13);
    
    % air data computer and engine model
    atmosphere = f16_adc(alt,Vt);
    cpow = f16_tgear(thtl);
    xdot(13) = f16_Pdot(pow,cpow);
    T = f16_thrust(pow,alt,atmosphere.M);
    
    % calculate force and moment coefficients
    TVt = 0.5/Vt;
    B2V = dims.b*TVt;
    Cq = dims.cbar*q*TVt;
    
    Ci_vec = f16_aeroFM(alpha,beta,ail,elev,rdr);
    damp_vec = f16_aerodamp(alpha);
    dMctrl_vec = f16_controlFM(alpha,beta);
    
    Cxq = damp_vec(1);
    Cyr = damp_vec(2);
    Cyp = damp_vec(3);
    Czq = damp_vec(4);
    Clr = damp_vec(5);
    Clp = damp_vec(6);
    Cmq = damp_vec(7);
    Cnr = damp_vec(8);
    Cnp = damp_vec(9);
    
    dlda = dMctrl_vec(1);
    dldr = dMctrl_vec(2);
    dnda = dMctrl_vec(3);
    dndr = dMctrl_vec(4);
    
    dail = ail/20;
    drdr = rdr/30;
    
    Cx = Ci_vec(1) + Cq*Cxq;
    Cy = Ci_vec(2) + B2V * (Cyr*r + Cyp*p);
    Cz = Ci_vec(3) + Cq*Czq;
    Cl = Ci_vec(4) + B2V*(Clr*r + Clp*p) + dlda*dail + dldr*drdr;
    Cm = Ci_vec(5) + Cq*Cmq + Cz*(dims.xcgr-xcg);
    Cn = Ci_vec(6) + B2V*(Cnr*r + Cnp*p) ...
        - Cy*(dims.xcgr - xcg)*(dims.cbar/dims.b) + dnda*dail + dndr*drdr;
    
    CB = cos(x(3));
    u = Vt*cos(x(2))*CB;
    v = Vt*sin(x(3));
    w = Vt*sin(x(2))*CB;
    
    qS = atmosphere.qbar*dims.S;
    qSb = qS*dims.b;
    m = mass.weight/mass.g;
    
    % Force equations
    Ay = qS/m*Cy;
    Az = qS/m*Cz;
    udot = r*v - q*w - mass.g*sin(theta) + (qS*Cx + T)/m;
    vdot = p*w - r*u + mass.g*cos(theta)*sin(phi) + Ay;
    wdot = q*u - p*v + mass.g*cos(theta)*cos(phi) + Az;
    xdot(1) = (u*udot + v*vdot + w*wdot)/Vt;
    xdot(2) = (u*wdot - w*udot)/(u^2 + w^2);
    xdot(3) = (Vt*vdot - v*xdot(1))*CB/(u^2 + w^2);
    
    % Kinematics
    xdot(4) = p + (sin(theta)/cos(theta))*(q*sin(phi) + r*cos(phi));
    xdot(5) = q*cos(phi) - r*sin(phi);
    xdot(6) = (q*sin(phi) + r*cos(phi))/cos(theta);
    
    % Moments
    roll = qSb * Cl;
    pitch = qS * dims.cbar * Cm;
    yaw = qSb * Cn;
    xPQ = mass.Jxz*(mass.Jxx - mass.Jyy + mass.Jzz);
    xQR = mass.Jzz*(mass.Jzz-mass.Jyy) + mass.Jxz*2;
    yPR = mass.Jzz - mass.Jxx;
    zPQ = (mass.Jxx - mass.Jyy)*mass.Jxx + mass.Jxz^2;   
    GAM = mass.Jxx*mass.Jzz - mass.Jxz^2;
    xdot(7) = (xPQ*p*q - xQR*q*r + mass.Jzz*roll + mass.Jxz*(yaw + q*mass.Hx))/GAM;
    xdot(8) = (yPR*p*r - mass.Jxz*(p^2 - r^2) + pitch - r*mass.Hx)/mass.Jyy;
    xdot(9) = (zPQ*p*q - xPQ*q*r + mass.Jxz*roll + mass.Jxx*(yaw + q*mass.Hx))/GAM;
    
    % Navigation    
    xdot(10) = u * cos(theta)*cos(psi) + ...
        v * (sin(phi)*cos(psi)*sin(theta) - cos(phi)*sin(psi)) +...
        w * (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)); % North speed
    xdot(11) = u * cos(theta)*sin(psi) + ...
        v * (sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi)) + ...
        w * (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)); % East speed
    xdot(12) = u * sin(theta) - v * sin(phi)*cos(theta) - ...
        w * cos(phi)*cos(theta);      % Vertical speed
    
%     An = -Az/mass.g;    %normal accleration (g's)
%     Alat = Ay/mass.g;   %lateral accleration (g's)
end

