% THIS IS TO SEE THE PERFORMANCE OF A CONTROLLER TRIMMED AROUND SOME
% CONDITION, IN EVERY OTHER CONDITION.

%% GETTING THE 'ONLY' CONTROLLER FIRST
% Additional parameters:
W = 2.5;
XCG = 0.33;
% Trim conditions
state = 2;
states = [10, 200];
% V = states(state,1);       
% H = states(state,2);  % ESTO ERA SÍ, YES, NO SÉ DE DÓNDE.
V = states(1);
H = states(2);
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% LQR Controller:
Q = blkdiag(1, 100, 100, 100, 10);
R = blkdiag(100 ,500);
K = lqr(A,B,Q,R);

% Perturbation:
StopTime = 5;
x0 = [-1, .0, .5, 0.1, -1.5];
sim("UAVLinCL_DEF_v0.slx")


%% NOW UN BUCLECITO DE PUTA MADRE:
% General settings for trim:
U0=[0.5;-0.1];
Y0=[];
IX=[1 4 5];
IU=[];
IY=[];

% Range of Flight Envelope:
vV = 10:1:35;
vH = 100:300:3100;

% Initial conditions: (BASIC ONE)
StopTime = 5;
x0 = [-1, .0, .5, 0.1, -1.5];

SIMDATA(length(vV), length(vH), 3) = outX;
    % (i,j) as in PI, for (V,H)
    % third index: 1-state, 2-incremental control, 3-real control.

Q = blkdiag(1, 100, 100, 100, 10);
R = blkdiag(100, 500);

for i = 1:length(vV)
    for j = 1:length(vH)
        disp(['(V,H) = (' num2str(i) '/' num2str(length(vV)) ', ' num2str(j) '/' num2str(length(vH)) ')'])
        % Conditions
        V = vV(i);
        H = vH(j);
        X0=[V;0;0;0;H];
        % Trim:
        [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
        % Linear Model:
        [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
        % with the K done above, simulating:
        K = lqr(A,B,Q,R);
        % Simulation:
        sim("UAVLinCL_DEF_v0");

        SIMDATA(i,j,1) = outX;
        SIMDATA(i,j,2) = outU;
        SIMDATA(i,j,3) = outUreal;
        
    end
end

%%
name_backup = 'BackUp_Simulation_Dataset_008.mat';
save(name_backup, "SIMDATA", "vV", "vH", "x0", 'Q', 'R')



%%
%% Generation of the dataset:
name_dataset = '../Datasets/dataset_008.csv';
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


%%
% [Xplot, Yplot] = meshgrid(vV, vH);
% figure(1)
% surf(Xplot,Yplot,PI')
% xlabel('Velocidad, V [m/s]')
% ylabel('Altura, H [m]')
% 
% figure(2)
% contour(Xplot, Yplot, PI')
% xlabel('Velocidad, V [m/s]')
% ylabel('Altura, H [m]')
% 
% figure(3)
% pcolor(Xplot, Yplot, PI')
% shading interp;
% colorbar;
% colormap("summer")
% xlabel('Velocidad, V [m/s]')
% ylabel('Altura, H [m]')
%%
figure(70)
Nlevels = 25;
% [M,cont] = contourf(vV, vH, PI','ShowText','on');
[M,cont] = contourf(vV, vH, PI', Nlevels);
% [M,cont] = contourf(vV, vH, PI);
colorbar;
% cont.LineWidth = 0
% cont.LevelStep = .02 % loque venía por defecto para el primer test
cont.LineStyle = 'none';
colormap("turbo")
% colormap("autumn")
xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)


