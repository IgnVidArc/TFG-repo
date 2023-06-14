function [] = SideBySide(outs, comps)

    if size(outs(1).Data, 2) == 5
        whole_leg = {'$V$', '$\alpha$', '$\theta$', '$q$', '$H$'};
    else
        whole_leg = {'$\delta_t$', '$\delta_e$'};
    end
    
    N = length(outs);
    figure()
    for k = 1:N
        subplot(1,N,k)
        out = outs(k);
        plot(out.Time, out.Data(:,comps), LineWidth=1)
        legend(whole_leg(comps), Interpreter='latex')
        hold on
    end


    

%     hold off
end