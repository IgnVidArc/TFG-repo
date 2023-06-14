function [] = mylegend(key, position, nfig)
    
    if nargin < 2
        position = 'northeast';
    end

    if nargin < 3
        nfig = 1;
    end

    if nfig==1
        legfontsize = 12;
    elseif nfig==2
        legfontsize = 15;
    else
    end

    numcolumns = 1;

    if contains(key, 'deltasboth')
        leg = {'$\delta_t$', '$\delta_e$',...
                '$(\delta_t)_{nn}$ [--]', '$(\delta_e)_{nn}$ [rad]'};
        numcolumns = 2;
    elseif contains(key, 'deltas')
        leg = {'$\delta_t$ [--]', '$\delta_e$ [rad]'};
    elseif contains(key, 'relus')
        leg = {'ReLU', 'LeackyReLU ($a = 0.05$)', ...
            'LeackyReLU ($a = 0.10$)', 'LeackyReLU ($a = 0.20$)'};        
    elseif contains(key, 'vyref')
        leg = {'$V$', '$r_v(t)$'};
    elseif key ==  'v'
        leg = {'$V$'};
    elseif contains(key, 'hyref')
        leg = {'$H$', '$r_h(t)$'};
    elseif key == 'h'
        leg = {'$H$'};
    elseif contains(key,'allxboth')
        % leg = {'$V$ [m/s]', '$\alpha$ [rad]', '$\theta$ [rad]', '$q$ [rad/s]', '$H$ [m]', ...
        leg = {'$V$', '$\alpha$', '$\theta$', '$q$', '$H$', ...
            '$(V)_{nn}$ [m/s]', '$(\alpha)_{nn}$ [rad]', '$(\theta)_{nn}$ [rad]', '$(q)_{nn}$ [rad/s]', '$(H)_{nn}$ [m]'};
        numcolumns = 2;
    elseif contains(key,'allx')
        leg = {'$V$ [m/s]', '$\alpha$ [rad]', '$\theta$ [rad]', '$q$ [rad/s]', '$H$ [m]'};
    elseif contains(key,'2345')
        leg = {'$\alpha$ [rad]', '$\theta$ [rad]', '$q$ [rad/s]', '$H$ [m]'};
    elseif contains(key,'15')
        leg = {'$V$ [m/s]', '$H$ [m]'};
    elseif contains(key,'234')
        leg = {'$\alpha$ [rad]', '$\theta$ [rad]', '$q$ [rad/s]'};
    elseif contains(key, '5')
        leg = {'$H$'};
    end


    legend(leg, Interpreter='latex', FontSize=legfontsize, Location=position, ...
        NumColumns=numcolumns)