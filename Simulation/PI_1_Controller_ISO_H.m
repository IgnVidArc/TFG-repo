% THIS IS TO SEE THE PERFORMANCE OF A CONTROLLER TRIMMED AROUND SOME
% CONDITION, IN EVERY OTHER CONDITION (MAINTAINING SAME HEIGHT)

%% GETTING THE 'ONLY' CONTROLLER FIRST
% Additional parameters:
W = 2.5;
XCG = 0.33;
% Trim conditions
V = 10;       
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% LQR Controller:
Q = blkdiag(1, 1, 1, 1, 1);
R = blkdiag(1,1);
[K,S,P] = lqr(A,B,Q,R);


%% J ERROR FROM LQR:
outbunch = sim("UAVLinCL_DEF_v0");
PI = PerformanceIndex(outX, outU, Q, R);
% plot(outX)


%% NOW UN BUCLECITO DE PUTA MADRE (AMBOS V y H: varias isoH):
%% CON EL CONTROLADOR DEFINIDO EN LA SECCIÓN SUPERIOR
% General settings for trim:
U0=[0.6;-0.1];
Y0=[];
IX=[1 4 5];
IU=[];
IY=[];

% % LQR controller cost parameters:
Q = blkdiag(1, 1, 1, 1, 1);
R = blkdiag(1,1);

% Range of Flight Envelope:
vV = 10:1:30;
vH = 100:400:3100;
% vH = [1000, 1500, 2000];
leg = cell(length(vH),1);
% Initial conditions:
x0 = [-1,0,0,0,0]';

PI = zeros(length(vV), length(vH));
for i = 1:length(vV)
    for j = 1:length(vH)
        leg{j} = ['$H = ' num2str(vH(j)) '$ m'];
        disp(['(V,H) = (' num2str(i) '/' num2str(length(vV)) ', ' num2str(j) '/' num2str(length(vH)) ')'])
        % Conditions
        V = vV(i);
        H = vH(j);
        X0=[V;0;0;0;H];
        % Trim:
        [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
        % Linear Model:
        [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
        % Controller: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [K,S,P] = lqr(A,B,Q,R);
        % Simulation:
        sim("UAVLinCL_DEF_v0");
        % PI:
        PI(i,j) = PerformanceIndex(outX, outU, Q, R);

    end
end

%% <HEATMAP> PI EN LA ENVULETA DE VUELO
figure(69)
Nlevels = 25;
% [M,cont] = contourf(vV, vH, PI','ShowText','on');
[M,cont] = contourf(vV, vH, PI', Nlevels);
% [M,cont] = contourf(vV, vH, PI);
colorbar;
% cont.LineWidth = 0
% cont.LevelStep = .02 % loque venía por defecto para el primer test
cont.LineStyle = 'none';
colormap(flipud(turbo))
% colormap("autumn")
xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)


%% PLOT OF PI AT DIFFERENT HEIGHTS:
figure()
% [M,cont] = contourf(vV, vH, PI','ShowText','on');
% [M,cont] = contourf(vV, vH, PI', Nlevels);
p = plot(vV, PI);
xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
ylabel('Performance Index [-]', Interpreter='latex', FontSize=12)
legend(leg, Interpreter='latex')

%% RESPONSES AT DIFF V : ISO HEIGHT:
% Iso-height
H = 100;
% Velocities to study:
vVbunch = [12, 15, 25];
Ncases = length(vVbunch);

x0 = [-5,0,0.05,0,0]';

sim("UAVLinCL_DEF_v0");
outbunch = outU;
outbunch(length(vVbunch),3) = outU; % just to initialize it
% Structure with first index for the iso-H points above different velocities.
% Second index is 1 for state X and 2 for incremental control U, 3 is for real U.
%%
tit = cell(Ncases);
PIbunch = zeros(Ncases);
for k = 1:Ncases
    tit{k} = ['$V =$ ' num2str(vVbunch(k)) ' m/s'];
    % Conditions
    V = vVbunch(k);
    X0=[V;0;0;0;H];
    % Trim:
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
    % Linear Model:
    [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
    % Simulation:
    sim("UAVLinCL_DEF_v0");
    % PI:
    PIbunch(k) = PerformanceIndex(outX, outU, Q, R);
    % SAVING THE RESPONSES:
    outbunch(k,1) = outX;
    outbunch(k,2) = outU;
    outbunch(k,3) = outUreal;
end

%% PLOTTING INCREMENTAL STATE AND CONTROL VECTOR
control = 2;     % 2: incremental control, 3: real control

f = figure(100);
for k = 1:Ncases
    subplot(2, Ncases, k)
    plot(outbunch(k,1))
    subplot(2, Ncases, Ncases+k)
    plot(outbunch(k,control))
end

xx=300; yy=80;
width = 1000;
height = 650;
set(f,'position',[xx,yy,width,height])

%% Plot of side-by-side responses:
caseA = 1;
caseB = 2;
% State:
QuickCompareSims([outbunch(caseA,1), outbunch(caseA,1)], 1:5)
% Control:
QuickCompareSims([outbunch(caseB,2), outbunch(caseB,2)], 1:2)
%%
function [] = QuickCompareSims(outs, comp)
    N = length(outs);
    leg = cell(N, length(comp));
    figure()
    for k = 1:N
        leg{k} = ['Case: ' num2str(k)];
        out = outs(k);
        plot(out.Time, out.Data(:,comp), LineWidth=1)
        hold on
    end
    legend(leg)
    xlabel('Tiempo [s]')
    ylabel(['X(' num2str(comp) ')'])
    grid on
%     hold off
end



