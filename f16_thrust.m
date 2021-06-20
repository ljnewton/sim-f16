function thrust = f16_thrust(pow,alt,Mach)
    
    alt = max(0,alt);
    
    altitude_vec = 0:10000:50000; % top to bottom
    Mach_vec = linspace(0,1,6); % left to right
    pow_vec = [0 50 100];

    % low throttle lookup
    A = [1060.0, 670.0, 880.0, 1140.0, 1500.0, 1860.0;
        635.0, 425.0, 690.0, 1010.0, 1330.0, 1700.0;
        60.0, 25.0, 345.0, 755.0, 1130.0, 1525.0;
        -1020.0, -710.0, -300.0, 350.0, 910.0, 1360.0;
        -2700.0, -1900.0, -1300.0, -247.0, 600.0, 1100.0;
        -3600.0, -1400.0, -595.0, -342.0, -200.0, 700.0];

    % mil power
    B = [12680.0, 9150.0, 6200.0, 3950.0, 2450.0, 1400.0;
        12680.0, 9150.0, 6313.0, 4040.0, 2470.0, 1400.0;
        12610.0, 9312.0, 6610.0, 4290.0, 2600.0, 1560.0;
        12640.0, 9839.0, 7090.0, 4660.0, 2840.0, 1660.0;
        12390.0, 10176.0, 7750.0, 5320.0, 3250.0, 1930.0;
        11680.0, 9848.0, 8050.0, 6100.0, 3800.0, 2310.0];

    % max power lookup (afterburner)
    C = [20000.0, 15000.0, 10800.0, 7000.0, 4000.0, 2500.0;
        21420.0, 15700.0, 11225.0, 7323.0, 4435.0, 2600.0;
        22700.0, 16860.0, 12250.0, 8154.0, 5000.0, 2835.0;
        24240.0, 18910.0, 13760.0, 9285.0, 5700.0, 3215.0;
        26070.0, 21075.0, 15975.0, 11115.0, 6860.0, 3950.0;
        28886.0, 23319.0, 18300.0, 13484.0, 8642.0, 5057.0];
    
    table = cat(3,A',B',C');
    thrust = interpn(altitude_vec,Mach_vec,pow_vec,table,alt,Mach,pow);

%     % find lookup coefficients I (altitude index) and M (Mach index)
%     H = alt/10000;
%     I = floor(H);
%     if I >= 6
%         I = 5;
%     end
%     dH = H - I;
% 
%     rM = 6*Mach;
%     M = floor(rM);
%     if M >= 6
%         M = 5;
%     end
%     dM = rM - M;
%     cdH = 1 - dH;
% 
%     % lookup military power as function of altitude, Mach
%     S = B(I,M) *cdH + B(I+1,M)*dH;
%     T = B(I,M+1)*cdH + B(I+1,M+1)*dH;
%     Tmil = S + (T-S)*dM;
% 
%     if pow < 50
%         % interpolate between zero and military power
%         S = A(I,M) *cdH + A(I+1,M)*dH;
%         T = A(I,M+1)*cdH + A(I+1,M+1)*dH;
%         Tidl = S + (T-S)*dM;
%         thrust = Tidl + (Tmil-Tidl)*pow*0.02;
%     else
%         % interpolate between military power and full afterburner
%         S = C(I,M)*cdH + C(I+1,M)*dH;
%         T = C(I,M+1)*cdH + C(I+1,M+1)*dH;
%         Tmax = S + (T-S)*dM;
%         thrust = Tmil + (Tmax - Tmil)*(pow-50)*0.02;
%     end

end