if nfig==1
    legfontsize = 12;
elseif nfig==2
    legfontsize = 15;
else
end

figure(2)
plot(outU.Time, outU.Data, LineWidth=1)
legend({'$\delta_t$ [--]', '$\delta_e$ [rad]'}, Interpreter='latex', ...
    FontSize=legfontsize, Location='southeast')
myxlabel('t')
myylabel('allu')
myplotformat