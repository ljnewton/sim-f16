function tgear = f16_tgear(thtl)

if thtl <= 0.77
    tgear = 64.94*thtl;
else
    tgear = 217.38*thtl - 117.38;
end

end