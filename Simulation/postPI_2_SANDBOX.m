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
Q = blkdiag(1, 1000, 1000, 100, 1);
% Q = blkdiag(1, 1, 1, 1, 1);
R = blkdiag(1,1);
[K,S,P] = lqr(A,B,Q,R);


%% SIMPLE SIMULATION:
% outbunch = sim("UAVLinCL_DEF_v0");
x0 = [1, 0, -.5, -.5, 0]';
x0 = [1, 0, 0, 0, 0]';
StopTime = 5
out = sim("UAVLinCL_DEF_v0");

figure()
SetAxisLatex(gca)
subplot(1,2,1)
plot(out, outX.Data)
legend({'$V$', '$\alpha$', '$\theta$', '$q$', '$H$'}, Interpreter='latex')
subplot(1,2,2)
plot(out, outU.Data)
legend({'$\delta_t$', '$\delta_e$'}, Interpreter='latex')

% xx=250*4/3; yy=150*4/3; width = 1000*3/4; height = 500*3/4;
set(gcf,'position',[381,330.6,750.4000000000001,375.2])
Q0 = blkdiag(1, 1, 1, 1, 1);
R0 = blkdiag(1,1);
PI0 = PerformanceIndex(outX, outU, Q0, R0)
PI = PerformanceIndex(outX, outU, Q, R)



%% PLOT OF RESPONSES AT DIFFERENT VELOCITIES:
% Iso-height
H = 100;
% Velocities to study:
vVbunch = [12, 15, 25];
Ncases = length(vVbunch);

x0 = [5, 0, 0, 0, 0]';

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
QuickCompareSims([SIMDATA(1,1,1)], 1:5)
% Control:
QuickCompareSims([SIMDATA(1,1,3)], 1:2)
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



