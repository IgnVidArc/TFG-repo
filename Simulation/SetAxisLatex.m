function [] = SetAxisLatex(ax)
    % Sets the xticklabels and yticklabels to latex interpreter of the 'ax' axis.
    axisfontsize = 14;

    xaxisproperties= get(ax, 'XAxis');
    xaxisproperties.TickLabelInterpreter = 'latex';
    xaxisproperties.FontSize = axisfontsize;

    yaxisproperties= get(ax, 'YAxis');
    yaxisproperties.TickLabelInterpreter = 'latex';
    yaxisproperties.FontSize = axisfontsize;
end