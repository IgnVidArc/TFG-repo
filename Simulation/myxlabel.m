function [] = myxlabel(type)
    if type=='v'
        xlabel("Velocidad, $V$ [m/s]")
    elseif type=='t'
        xlabel("Tiempo, $t$ [s]")
    elseif type=='x'
        xlabel("x")
    end