function atmosphere = f16_adc(alt,Vt)
    % Air data computer
    % adapted from Lewis & Stevens, Aircraft Control and Simulation

    rho0 = 2.377e-3;    % sea level density
    
    T_fac = 1 - 0.703e-5.*alt;
    T = 519 * T_fac;
    
    if alt >= 35000
        T = 390 + 0*T;
    end
    
    rho = rho0*(T_fac.^4.14);       % air density
    M = Vt ./ sqrt(1.4*1716.3*T);   % Mach 
    
    qbar = 0.5*rho*Vt*Vt;   % dynamic pressure
    Ps = 1715 * rho.*T;     % static pressure
    
    atmosphere.T = T;
    atmosphere.rho = rho;
    atmosphere.M = M;
    atmosphere.qbar = qbar;
    atmosphere.Ps = Ps;

end