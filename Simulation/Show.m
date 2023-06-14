figure()
subplot(1,2,1)
plot(outX.Time, outX.Data)
legend({'$V$', '$\alpha$', '$\theta$', '$q$', '$H$'}, Interpreter='latex', FontSize=12)
myplotformat
subplot(1,2,2)
plot(outU.Time, outU.Data)
legend({'$\delta_t$', '$\delta_e$'}, Interpreter='latex', FontSize=12)
myplotformat
% f = gcf;
% % f.Position = [10, 200, 1500, 420];