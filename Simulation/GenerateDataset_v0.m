%% GENERATION OF THE DATASET
% Additional parameters:
W = 2.5;
XCG = 0.33;
% Trim conditions
V = 12;       
H = 2500;
X0=[V;0;0;0;H]; U0=[0.6;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% Perturbation de CTA:
x0 = [-1,0,.05,0,0]';

% LQR Controller:
Q = blkdiag(1, 1, 1, 1, 1);
R = blkdiag(1,1);
[K,S,P] = lqr(A,B,Q,R);

%% Simulation:
sim("UAVLinCL_DEF_v0");
% % % outX = outX;
% % % outU = outU;

%% J ERROR FROM LQR:
PI = PerformanceIndex(outX, outU, Q, R);


%% NOW UN BUCLECITO DE PUTA MADRE:
% General settings for trim:
U0=[0.6;-0.1];
Y0=[];
IX=[1 4 5];
IU=[];
IY=[];

% LQR controller cost parameters:
Q = blkdiag(1, 1, 1, 1, 1);
R = blkdiag(1,1);

% Range of Flight Envelope:
vV = 10:2:30;
vH = 100:500:3100;

% Initial conditions:
x0 = [-1,0,0,0,0]';

PI = zeros(length(vV), length(vH));
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
        % Controller: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [K,S,P] = lqr(A,B,Q,R);
        % Simulation:
        sim("UAVLinCL_DEF_v0");
        % PI:
        PI(i,j) = PerformanceIndex(outX, outU, Q, R);

    end
end

% with:
% vV = 10:2:30;
% vH = 100:500:3100;
% % (V,H) = (5/11, 2/7)
% % Warning: Matrix is close to singular or badly scaled. Results may be inaccurate. RCOND =  1.427781e-23.
% % (V,H) = (5/11, 4/7)
% % Warning: Matrix is close to singular or badly scaled. Results may be inaccurate. RCOND =  5.493058e-25. 

%%
figure(4)
Nlevels = 25;
% [M,cont] = contourf(vV, vH, PI','ShowText','on');
[M,cont] = contourf(vV, vH, PI', Nlevels);
colorbar;
% cont.LineWidth = 0
% cont.LevelStep = .02 % loque venía por defecto para el primer test
cont.LineStyle = 'none';
colormap("turbo")
% colormap("autumn")
xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=12)
ylabel('Altura, $H$ [m]', Interpreter='latex', FontSize=12)





