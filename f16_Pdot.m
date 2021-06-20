function Pdot = f16_Pdot(P3,P1)
    % P3: actual power
    % P1: power command
    
    if P1 >= 50
        if P3 >= 50
            T = 5;
            P2 = P1;
        else
            P2 = 60;
            T = f16_Rtau(P2-P3);
        end
    else
        if P3 >= 50
            T = 5;
            P2 = 40;
        else
            P2 = P1;
            T = f16_Rtau(P2-P3);
        end
    end
    
    Pdot = T*(P2-P3);

end

function Rtau = f16_Rtau(dP)
    % reciprocal time constant
    if dP <= 25
        Rtau  = 1;
    elseif dP >= 50
        Rtau = 0.1;
    else
        Rtau = 1.9 - 0.036*dP;
    end
end