% THIS IS TO SEE THE PERFORMANCE OF A CONTROLLER TRIMMED AROUND SOME
% CONDITION, IN EVERY OTHER CONDITION.

%% GETTING THE 'ONLY' CONTROLLER FIRST
% Additional parameters:
W = 2.5;
XCG = 0.33;
% Trim conditions
V = 30;       
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);


% LQR Controller:
Q = blkdiag(1, 1000, 1000, 100, 1);
R = blkdiag(1,1) * 1000;
[K,S,P] = lqr(A,B,Q,R);

%%

x0 = [0,0,0,0,2]';
StopTime = 20;
sim("UAVLinCL_DEF_v0.slx")
Show
%%
figure
plot(outU.Time, outU.Data(:,2))
xlim([0,1])



%% LOOP TO COVER OF THE FLIGHT ENVELOPE:
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
vV = 10:2:30;
vH = 100:400:3100;

% Initial conditions: (BASIC ONE)
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
        % Simulation:
        sim("UAVLinCL_DEF_v0");
        % PI:
        PI(i,j) = PerformanceIndex(outX, outU, Q, R);

    end
end
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
figure(5)
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


