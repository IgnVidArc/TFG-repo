%% READING A FILE WITH THE TESTING CONDITIONS AND PREDICTED CONTROLLERS AND SHOWING THEIS PI
% Additional parameters:
W = 2.5;
XCG = 0.33;

%% READING CONDITIONS AND CONTROLLERS' COMPONENTS:
predicted_controllers_name = 'PredictedControllers_002_test1.csv';
DATA = readmatrix(['../Controllers/' predicted_controllers_name]);

%%
vV = zeros(length(DATA),1);
vH = zeros(length(DATA),1);

x0 = [-1, 0, -.5, -.5, 0]';

Q0 = blkdiag(1, 100, 100, 0, 0);
R0 = blkdiag(1, 1) * 1;

predictedSIMDATA(length(DATA), 3) = outX; % Only inicializing it

PI_0 = zeros(length(DATA),1);
for k = 1:length(DATA)
    disp(['(k) = (' num2str(k) '/' num2str(length(DATA)) ')'])
    % Velocity and Height:
    V = DATA(k,1); vV(k) = V;
    H = DATA(k,2); vH(k) = H;
    % Controller:
    K = reshape(DATA(k,3:12), 2, 5);
    % System:
    X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
    IX=[1 4 5];     IU=[];         IY=[];
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
    sim("UAVLinCL_DEF_v0");

    predictedSIMDATA(k,1) = outX;
    predictedSIMDATA(k,2) = outU;
    predictedSIMDATA(k,3) = outUreal;

    PI_0(k) = PerformanceIndex(outX, outU, Q0, R0);
end

%% REORGANISING VECTORS FOR CONTOURF

uniqueV = unique(vV);
uniqueH = unique(vH);

PI_mat = NaN * ones(length(uniqueV), length(uniqueH));
for k = 1:length(DATA)
    v = DATA(k,1);
    h = DATA(k,2);
    idxV = find(v==uniqueV);
    idxH = find(h==uniqueH);
    PI_mat(idxV, idxH) = PI_0(k);
end

%% WANT: COLORMAP
ff = figure(778);
Nlevels = 45;
ss = surf(uniqueV, uniqueH, PI_mat');
colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
myxlabel('v')
myylabel('h')
myplotformat
cb = ax.Colorbar;
cb.TickLabelInterpreter = 'latex';
cb.FontSize = 12;

%% PILLAR UNA LÍNEA DE 'DATA' IMPORTANDO LOS CONTROLADORES T VIENDO LA RESPUESTA:

[outX, outU, outUreal, V, H, PI] = DrawIndexFromData(DATA, 4)

function [outX, outU, outUreal, V, H, PI] = DrawIndexFromData(DATA, index)
    V = DATA(index,1); vV(index) = V;
    H = DATA(index,2); vH(index) = H;
    % Controller:
    K = reshape(DATA(index,3:12), 2, 5);
    % System:
    X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
    IX=[1 4 5];     IU=[];         IY=[];
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);


    StopTime = 10;
    sim("UAVLinCL_DEF_v0");

    Q0 = blkdiag(1, 100, 100, 0, 0);
    R0 = blkdiag(1, 1) * 1;
    PI = PerformanceIndex(outX, outU, Q0, R0);

    Show
    disp(['(V,H) = (' num2str(V) ', ' num2str(H) ')'])
    disp(['PI = ' PI])
end


