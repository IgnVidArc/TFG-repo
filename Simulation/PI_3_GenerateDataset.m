% PI_3_GeneralteDataset.m
%% DATASET GENERATION

% Additional parameters:
W = 2.5;
XCG = 0.33;
IP = 0.1568;% Moment of inertia [kg m^2]
% Trim conditions
V = 15.2;
H = 1000;
X0=[V;0;0;0;H]; U0=[1;0.5]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
%
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% LQR Controller:
Q = blkdiag(1, 1000, 1000, 100, 10);
R = blkdiag(1,1) * 100;
[K,S,P] = lqr(A,B,Q,R);


%% SIMPLE SIMULATION:
% outbunch = sim("UAVLinCL_DEF_v0");
x0 = [-1, 0, -.5, -.5, 0]';
StopTime = 5*2;
sim("UAVLinCL_DEF_v0");
Show

%% BUCLE DE ENVOLVENTE DE VUELO
% Valores definidos de velocidad (vV) y altura (vH).

% General settings for trim:
U0=[1;0.5]; Y0=[]; IX=[1 4 5]; IU=[]; IY=[];
% Range of Flight Envelope:
% vV = 10:5:30; %vV = 10:1:30;
% vH = 100:500:3100; %vH = 100:500:3100;

vV = linspace(10,35,3);
vH = linspace(0,6000,3);

leg = cell(length(vH),1);
% Initial conditions:
% x0 = [-1, 0, -.5, -.5, 0]';
% x0 = [-1, 0, 0, 0, 0]';

% Different Performance Indexes:
    % Original controller:
    PI = zeros(length(vV), length(vH));
    Q = blkdiag(1, 100, 100, 100, 10);
    R = blkdiag(100, 500);
    % With the identities:
    PI_I = zeros(length(vV), length(vH));
    QI = eye(5);
    RI = eye(2);
    % With a reference Q0, R0:
    PI_0 = zeros(length(vV), length(vH));
    Q0 = blkdiag(1, 100, 100, 0, 0);
    R0 = blkdiag(1, 1) * 100;
    % With reference but with the new PI (abs, increm):
    PI_0mod = zeros(length(vV), length(vH));

% Recollection of responses:
% SIMDATA(length(vV), length(vH), 3) = outX; % (initialization)
    % (i,j) as in PI, for (V,H)
    % third index: 1-state, 2-incremental control, 3-real control.
% Recollection of Controller Weights:
CONTROLLER = zeros(length(vV), length(vH), 2, 5);

StopTime = 5;


% oftenV = 4;         % i-index jump for the V's to simulate
% oftenH = 4;         % j-index jump for the H's to simulate
% % every other case is calculated and generated a K,
% % but no simulation is done, and, consequently, no PI is calculated
% % Index for vV and vH to simulate:
% V_chosen_ones = [];
% H_chosen_ones = [];

for i = 1:length(vV)
    for j = 1:length(vH)
        leg{j} = ['$H = ' num2str(vH(j)) '$ m'];
        disp(['(V,H) = (' num2str(i) '/' num2str(length(vV)) ', ' num2str(j) '/' num2str(length(vH)) ')'])
        % Conditions
        V = vV(i);
        H = vH(j);
        X0=[V;0;0;0;H];
        % Trim:
        U0=[0.5;-0.1];
        [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
        % Linear Model:
        [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
        % Controller: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [K,S,P] = lqr(A,B,Q,R);
        % Simulation:
        % if mod(i,oftenV) == 0 && mod(j,oftenH) == 0
        %     V_chosen_ones = [V_chosen_ones; i];
        %     H_chosen_ones = [H_chosen_ones; j];
            % sim("UAVLinCL_DEF_v0");
            % % PI:
            % PI(i,j)  = PerformanceIndex(outX, outU, Q, R);
            % PI_I(i,j) = PerformanceIndex(outX, outU, QI, RI);
            % PI_0(i,j) = PerformanceIndex(outX, outU, Q0, R0);
            % PI_0mod(i,j) = PerformanceIndexMod(outX, outU, outUreal, Q0, R0);
            % % Saving the responses:
            % SIMDATA(i,j,1) = outX;
            % SIMDATA(i,j,2) = outU;
            % SIMDATA(i,j,3) = outUreal;
        % end
        CONTROLLER(i,j,:,:) = K;
    end
end

%% Save controller:
name_backup = 'BackUp_Simulation_Dataset_009.mat';
% save(name_backup, "SIMDATA", "CONTROLLER", "vV", "vH", "x0", 'PI_0', 'Q', 'R', 'Q0', 'R0')
save(name_backup, "CONTROLLER", "vV", "vH", 'Q', 'R')

%% Generation of the dataset:
name_dataset = '../Datasets/dataset_009.csv';
export = zeros(length(vV)*length(vH), 2 + 2*5);
idx = 0;
for i = 1:length(vV)
    for j = 1:length(vH)
        idx = idx + 1;
        export(idx,:) = [vV(i), vH(j), ...
            reshape(CONTROLLER(i,j,1,:),1,5), reshape(CONTROLLER(i,j,2,:),1,5)];
    end
end
writematrix(export,name_dataset,'Delimiter',',')
% disp('done')

%% NEW: COMPARISON BETWEEN THE NEW PI MODIFIED
% Nlevels = 45;
% figure(999)
% subplot(1,2,1)
%     [M,cont] = contourf(vV, vH, PI_0', Nlevels);
%     colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
%     xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
%     ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)
%     title('0')
% subplot(1,2,2)
%     [M_I,cont] = contourf(vV, vH, PI_0mod', Nlevels);
%     colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
%     xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
% %     ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)
%     title('0mod')
% f = gcf;
% f.Position = [10, 200, 1500, 420];

%%
oftenV = 4;         % i-index jump for the V's to simulate
oftenH = 4;         % j-index jump for the H's to simulate
V_chosen_ones = [];
H_chosen_ones = [];
PI_0_chosen_ones = [];
idxv = 0; idxh = 0;
for i = 1:length(vV)
    for j = 1:length(vH)
        if mod(i,oftenV) == 0 && mod(j,oftenH) == 0
            idxv = idxv + 1;
            idxh = idxh + 1;
            V_chosen_ones = [V_chosen_ones, vV(i)];
            H_chosen_ones = [H_chosen_ones, vH(j)];
            % PI_0_chose_ones(i,j) = PI_0(i,j)
            PI_0_chosen_ones(idxv, idxh) = PI_0(i,j);
        end
    end
end

%% WANT: COLORMAP
ff = figure(778);
Nlevels = 35;
have_simulated_all = 1;
if have_simulated_all
    [M,cont] = contourf(vV, vH, PI_0', Nlevels);
else
    [XX, YY] = meshgrid(V_chosen_ones, H_chosen_ones)
    [M,cont] = contourf(XX, YY, PI_0_chosen_ones', Nlevels);
end
colorbar; cont.LineStyle = 'none'; colormap(flipud(turbo))
myxlabel('v')
myylabel('h')
myplotformat
cb = ax.Colorbar;
cb.TickLabelInterpreter = 'latex';
cb.FontSize = 12;

%% NEW:
%% Plot of side-by-side responses:
indexV = 1;
indexH = 7;

% State:
QuickCompareSims([SIMDATA(indexV,indexH,1)], 1:5)
% Incremental Control:
QuickCompareSims([SIMDATA(indexV,indexH,2)], 1:2)

%%
% disp(length(vV))  --> 81
% disp(length(vH))  --> 31
va = 1; ha = 25;
vb = 81; hb = 25;

% State
SideBySide([SIMDATA(va,ha,1), SIMDATA(vb,hb,1)], 1:5)

% Incremental control
SideBySide([SIMDATA(va,ha,2), SIMDATA(vb,hb,2)], 1:2)


%% UPDATING THE PERFORMANCE INDEX WITH THE STORED SIMULATION DATA:

R0 = blkdiag([100, 100])

for i = 1:length(vV)
    for j = 1:length(vH)
        disp(['(V,H) = (' num2str(i) '/' num2str(length(vV)) ', ' num2str(j) '/' num2str(length(vH)) ')'])
        % PI:
        PI(i,j)  = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q, R);
        PI_I(i,j) = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), QI, RI);
        PI_0(i,j) = PerformanceIndex(SIMDATA(i,j,1), SIMDATA(i,j,2), Q0, R0);
    end
end

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




