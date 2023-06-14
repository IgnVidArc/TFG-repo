if nfig==1
    legfontsize = 12;
elseif nfig==2
    legfontsize = 15;
else
end

figure(1)
plot(outX.Time, outX.Data, LineWidth=1)
legend({'$V$ [m/s]', '$\alpha$ [rad]', '$\theta$ [rad]', '$q$ [rad/s]', '$H$ [m]'}, ...
    Interpreter='latex', FontSize=legfontsize, Location='southeast')
myxlabel('t')
myylabel('allx')
myplotformat