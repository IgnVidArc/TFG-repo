% load("BackUp_Simulation_cap42_A.mat")
% load("BackUp_Simulation_Dataset_005.mat")
load('BackUp_Simulation_cap42_cada1ms_V35_K.mat')


%% UPDATING THE PERFORMANCE INDEX WITH THE STORED SIMULATION DATA:

% PI es con el Q,R de nuestro nuevo controlador
PI = zeros(length(vV), length(vH));
PI_state = zeros(length(vV), length(vH));
PI_control = zeros(length(vV), length(vH));

PI_I = zeros(length(vV), length(vH));
PI_0 = zeros(length(vV), length(vH));

Q = blkdiag(1, 100, 100, 0, 0);
R = blkdiag(100, 100);
% Q_I = eye(5);
% R_I = eye(2);


for i = 1:length(vV)
    for j = 1:length(vH)
        disp(['(V,H) = (' num2str(i) '/' num2str(length(vV)) ', ' num2str(j) '/' num2str(length(vH)) ')'])
        % PI:
        PI(i,j)  = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q, R);
        PI_state(i,j)  = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q, 0*R);
        PI_control(i,j)  = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), 0*Q, R);

        % PI_I(i,j) = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q_I, R_I);
        % PI_0(i,j) = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q0, R0);
    end
end


%%
% close all
figure(4)
% subplot(1,3,1)
Nlevels = 25;
[M,cont] = contourf(vV, vH, PI', Nlevels);
hold on
colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
myxlabel('v')
myylabel('h')
nfig = 2;
myplotformat
cb = ax.Colorbar;
cb.TickLabelInterpreter = 'latex';
cb.FontSize = axisfontsize;
pointA = states(1,:);
pointB = states(2,:);
txtA = {'$\ $A'};
txtB = {'$\ $B'};
plot(pointA(1), pointA(2), 'o', LineWidth=1, Color='k')
text(pointA(1), pointA(2), txtA, Interpreter='latex', FontSize=axisfontsize)
plot(pointB(1), pointB(2), 'o', LineWidth=1, Color='white')
text(pointB(1), pointB(2), txtB, Interpreter='latex', FontSize=axisfontsize, Color='white')
% mysave('cap4_PI_K.pdf')
%%

% figure(5)
subplot(1,3,2)
Nlevels = 25;
[M,cont] = contourf(vV, vH, PI_state', Nlevels);
hold on
colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
myxlabel('v')
myylabel('h')
nfig = 2;
myplotformat
cb = ax.Colorbar;
cb.TickLabelInterpreter = 'latex';
cb.FontSize = axisfontsize;
pointA = states(1,:);
pointB = states(2,:);
txtA = {'$\ $A'};
txtB = {'$\ $B'};
plot(pointA(1), pointA(2), 'o', LineWidth=1, Color='k')
text(pointA(1), pointA(2), txtA, Interpreter='latex', FontSize=axisfontsize)
plot(pointB(1), pointB(2), 'o', LineWidth=1, Color='white')
text(pointB(1), pointB(2), txtB, Interpreter='latex', FontSize=axisfontsize, Color='white')

% figure(6)
subplot(1,3,3)
Nlevels = 25;
[M,cont] = contourf(vV, vH, PI_control', Nlevels);
hold on
colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
myxlabel('v')
myylabel('h')
nfig = 2;
myplotformat
cb = ax.Colorbar;
cb.TickLabelInterpreter = 'latex';
cb.FontSize = axisfontsize;
pointA = states(1,:);
pointB = states(2,:);
txtA = {'$\ $A'};
txtB = {'$\ $B'};
plot(pointA(1), pointA(2), 'o', LineWidth=1, Color='k')
text(pointA(1), pointA(2), txtA, Interpreter='latex', FontSize=axisfontsize)
plot(pointB(1), pointB(2), 'o', LineWidth=1, Color='white')
text(pointB(1), pointB(2), txtB, Interpreter='latex', FontSize=axisfontsize, Color='white')


