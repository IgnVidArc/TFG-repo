% Default plotting configuration
if nfig==1
    axisfontsize = 14;
elseif nfig==2
    axisfontsize = 17;
else
end

f = gcf;
ax = gca;

xaxisproperties= get(ax, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
xaxisproperties.FontSize = axisfontsize;

yaxisproperties= get(ax, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties.FontSize = axisfontsize;

tx = ax.XLabel;
tx.Interpreter = "latex";
ty = ax.YLabel;
ty.Interpreter = "latex";

box on
grid off