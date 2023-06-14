function [] = myylabel(type)
    if  contains(type,'habs')
        ylabel("Altura absoluta [m]")

    elseif contains(type, 'relus')
        ylabel("Funci\'on de activaci\'on")

    elseif type=='h'
        ylabel("Altura, $H$ [m]")

    elseif contains(type,'vabs')
        ylabel("Velocidad absoluta [m/s]")
    elseif type=='v'
        ylabel("Velocidad, $V$ [m/s]")
    
    elseif contains(type, 'x')
        ylabel("Variables de estado")
    elseif contains(type, 'uabs')
        ylabel("Variables absolutas de control")
    elseif contains(type, 'u')
        ylabel("Variables de control")
    end