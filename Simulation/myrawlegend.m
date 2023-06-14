function [] = myrawlegend(leg, nfig)
    

    if nargin < 2
        nfig = 1;
    end

    if nfig==1
        legfontsize = 12;
    elseif nfig==2
        legfontsize = 15;
    else
    end

    legend(leg, Interpreter='latex', FontSize=legfontsize)