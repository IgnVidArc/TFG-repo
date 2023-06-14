%% GETTING THE 'ONLY' CONTROLLER FIRST
% Additional parameters:
W = 2.5;
XCG = 0.33;
% Trim conditions
V = 15;       
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% LQR Controller:
Q = blkdiag(1, 1000, 1000, 100, 1);
R = blkdiag(1,1);
[K,S,P] = lqr(A,B,Q,R);


%% SIMPLE SIMULATION:
% outbunch = sim("UAVLinCL_DEF_v0");
x0 = [-5, 0, -.5, -.5, 0]';
out = sim("UAVLinCL_DEF_v0");
% PI = PerformanceIndex(outX, outU, Q, R);
plot(out, outUreal.Data)


%% BUCLE DE ENVOLVENTE DE VUELO
% Valores definidos de velocidad (vV) y altura (vH).

% General settings for trim:
U0=[0.6;-0.1]; Y0=[]; IX=[1 4 5]; IU=[]; IY=[];
% Range of Flight Envelope:
vV = 10:2:30;
vH = 100:500:3100;
% vH = [1000, 1500, 2000];
leg = cell(length(vH),1);
% Initial conditions:
x0 = [-1, 0, -.5, -.5, 0]';
% x0 = [-1, 0, 0, 0, 0]';

% Different Performance Indexes:
    % Original controller:
    PI = zeros(length(vV), length(vH));
    Q = blkdiag(1, 1000, 1000, 100, 1);
    R = blkdiag(1, 1);
    % With the identities:
    PI_I = zeros(length(vV), length(vH));
    QI = eye(5);
    RI = eye(2);
    % With a reference Q0, R0:
    PI_0 = zeros(length(vV), length(vH));
    Q0 = blkdiag(1, 100, 100, 0, 0);
    R0 = blkdiag(1, 1) * 1;
% Recollection of responses:
SIMDATA(length(vV), length(vH), 3) = outX;
    % (i,j) as in PI, for (V,H)
    % third index: 1-state, 2-incremental control, 3-real control.

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
%         Controller: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [K,S,P] = lqr(A,B,Q,R);
        % Simulation:
        sim("UAVLinCL_DEF_v0");
        % PI:
        PI(i,j)  = PerformanceIndex(outX, outU, Q, R);
        PI_I(i,j) = PerformanceIndex(outX, outU, QI, RI);
        PI_0(i,j) = PerformanceIndex(outX, outU, Q0, R0);
        % Saving the responses:
        SIMDATA(i,j,1) = outX;
        SIMDATA(i,j,2) = outU;
        SIMDATA(i,j,3) = outUreal;
    end
end

%% HEATMAP OF THE THREE PERFORMANCE INDEXES:
Nlevels = 45;

figure(1001)
subplot(1,3,1)
    [M,cont] = contourf(vV, vH, PI', Nlevels);
    colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
    xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
    ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)
    title('(Q,R): Controler')
subplot(1,3,2)
    [M_I,cont] = contourf(vV, vH, PI_I', Nlevels);
    colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
    xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
%     ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)
    title('(Q,R): Identity')
subplot(1,3,3)
    [M_0,cont] = contourf(vV, vH, PI_0', Nlevels);
    colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
    xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
%     ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)
    title('(Q,R): Reference')
f = gcf;
f.Position = [10, 200, 1500, 420];

%% UPDATING THE PERFORMANCE INDEX WITH THE STORED SIMULATION DATA:

for i = 1:length(vV)
    for j = 1:length(vH)
        disp(['(V,H) = (' num2str(i) '/' num2str(length(vV)) ', ' num2str(j) '/' num2str(length(vH)) ')'])
        % PI:
        PI(i,j)  = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q, R);
        PI_I(i,j) = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), QI, RI);
        PI_0(i,j) = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q0, R0);
    end
end


%% HEATMAP: PI EN LA ENVULETA DE VUELO:
% figure(69)
% Nlevels = 45;
% [M1,cont] = contourf(vV, vH, PI', Nlevels);
% colorbar;
% cont.LineStyle = 'none';
% colormap(flipud(turbo))
% xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
% ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)

%% HEATMAP: PI0 EN LA ENVULETA DE VUELO:
% figure(70)
% Nlevels = 45;
% % [M,cont] = contourf(vV, vH, PI','ShowText','on');
% [M2,cont] = contourf(vV, vH, PI_I', Nlevels);
% % [M,cont] = contoxurf(vV, vH, PI);
% colorbar;
% % cont.LineWidth = 0
% % cont.LevelStep = .02 % loque venía por defecto para el primer test
% cont.LineStyle = 'none';
% colormap(flipud(turbo))
% % colormap("autumn")
% xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
% ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)

%% PLOT OF PI AT DIFFERENT HEIGHTS:
figure(71)
% [M,cont] = contourf(vV, vH, PI','ShowText','on');
% [M,cont] = contourf(vV, vH, PI', Nlevels);
p = plot(vV, PI);
xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
ylabel('Performance Index [-]', Interpreter='latex', FontSize=12)
legend(leg, Interpreter='latex', Location='best')
grid on

%% PLOT OF RESPONSES AT DIFFERENT VELOCITIES:
% Iso-height
H = 100;
% Velocities to study:
vVbunch = [12, 15, 25];
Ncases = length(vVbunch);

x0 = [1, 0, -.5, -.5, 0]';

sim("UAVLinCL_DEF_v0");
outbunch = outU;
outbunch(length(vVbunch),3) = outU; % just to initialize it
% Structure with first index for the iso-H points above different velocities.
% Second index is 1 for state X and 2 for incremental control U, 3 is for real U.

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
f = figure(72);
for k = 1:Ncases
    subplot(2, Ncases, k)
    plot(outbunch(k,1))
    title([tit{k} ', State'], Interpreter='latex')
    subplot(2, Ncases, Ncases+k)
    plot(outbunch(k,control))
    title([tit{k} ', Control'], Interpreter='latex')
end

xx=300; yy=80; width = 1000; height = 650;
set(f,'position',[xx,yy,width,height])

%% Plot of side-by-side responses:
caseA = 1;
caseB = 2;
% State:
QuickCompareSims([outbunch(caseA,1), outbunch(caseA,1)], 1:5)
% Control:
QuickCompareSims([outbunch(caseB,2), outbunch(caseB,2)], 1:2)
%%
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
%     legend(legg)
%     xlabel('Tiempo [s]')
%     ylabel(['X(' num2str(comps) ')'])
    grid on
%     hold off
end



