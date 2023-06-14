function [] = QuickCompareSims(outs, comps)
    N = length(outs);
    legg = cell(N, length(comps));
    figure()
    for k = 1:N
        legg{k} = ['Case: ' num2str(k)];
        out = outs(k);
        plot(out.Time, out.Data(:,comps), LineWidth=1)
        hold on
    end
%     hold off
end