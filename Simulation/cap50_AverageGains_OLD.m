% cap5x_AverageGains.m

% [Wref, XCGref, IPref] = 2.5000    0.3300    0.1568
load("BackUp_Simulation_Dataset_5D_002.mat")

% vV   =        10        15        20        25        30      35
% vH   =         0       750      1500      2250      3000
% vW   =    1.8750    2.1875    2.5000    2.8125    3.1250
% vXCG =    0.2475    0.2888    0.3300    0.3713    0.4125
% vIP  =    0.1176    0.1372    0.1568    0.1764    0.1960

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ALL YOU NEED TO KNOW IS FROM HERE:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
c1 = [0 0.4470 0.7410];
c2 = [0.8500 0.3250 0.0980];
c3 = [0.9290 0.6940 0.1250];
c4 = [0.4940 0.1840 0.5560];
c5 = [0.4660 0.6740 0.1880];
c6 = [0.3010 0.7450 0.9330];
c7 = [0.6350 0.0780 0.1840];
c8 = [1 0 0];
c9 = [0 1 0];
c10 = [0 0 1];

% colororder([c1; c2; c3; c4; c9; c6; c7; c8; c4; c10])
colors = [c1; c2; c3; c4; c9; c6; c7; c8; c4; c10];
colores(1,1,:) = c1;
colores(1,2,:) = c2
colores(1,3,:) = c3
colores(1,4,:) = c4
colores(1,5,:) = c9
colores(2,1,:) = c6
colores(2,2,:) = c7
colores(2,3,:) = c8
colores(2,4,:) = c5
colores(2,5,:) = c10
%% ESTO SÍ
deltas = zeros(1,10);
UVES = zeros(6,10);
for i=1:10
    deltas(i) = -2 + (i-0)*4/10;
end
for i=1:6
    for j=1:10
        UVES(i,j) = vV(i) + deltas(j);
    end
end
%%
figure
hold on
for i=1:length(vV)
for j=1:length(vH)
for k=1:length(vW)
for l=1:length(vXCG)
for m=1:length(vIP)
    j
    for row=1:2
        for col=1:5
            idx = (row-1)*5+col;
            gain = CONTROLLER(i,j,k,l,m,row,col);
            % plot(vV(i), gain, 's', color=colores(row,col,:))
            plot(UVES(i,idx), gain, 's', color=colores(row,col,:))
        end
    end
end; end; end; end; end


yl = [-6, 8];
plot(12.5*[1,1], yl, linestyle = '--', color="#C3BFBF")
hold on
plot((12.5+1*5)*[1,1], yl, linestyle = '--', color="#C3BFBF")
plot((12.5+2*5)*[1,1], yl, linestyle = '--', color="#C3BFBF")
plot((12.5+3*5)*[1,1], yl, linestyle = '--', color="#C3BFBF")
plot((12.5+4*5)*[1,1], yl, linestyle = '--', color="#C3BFBF")
plot((12.5+5*5)*[1,1], yl, linestyle = '--', color="#C3BFBF")


legend({'$k_{11}$', '$k_{21}$', ...
        '$k_{12}$', '$k_{22}$', ...
        '$k_{13}$', '$k_{23}$', ...
        '$k_{14}$', '$k_{24}$', ...
        '$k_{15}$', '$k_{25}$',}, Interpreter="latex", NumColumns=2, Location='northwest', ...
        fontsize=12)

nfig = 1;
myxlabel('v')
ylabel('Ganancias de cotrolador', FontSize=13)
xlim([10-1, 35+1])
myplotformat

%%
mysave('cap5_averagegains.pdf')

