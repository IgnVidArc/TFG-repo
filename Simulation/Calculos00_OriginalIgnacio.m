%% Trim
% C select flight path angle (y=gamma)
% Param
W = 2.5;      % Mass
XCG = 0.33;   % CG position (xcg/cma)
V = 12;       % Velocidad aerodinámica
H = 100;      % Altitud

% [Vt alpha theta q  h]
X0=[V;0;0;0;H];
U0=[0.6;-0.1];
Y0=[];
IX=[1 4 5];
IU=[];
IY=[];
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);

aoa = Xtrim(2)*180/pi

%% Linealize

[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

eigsOL = eig(A)

%% Para el modelo NL
pert = [-2,0,0,0,-2]';
xi = Xtrim + pert;  % Este es el nombre que lleva lo que recibe UAVNLCL.

%% Controlador:
Q = blkdiag(1, 10, 10, 1, 1);
R = blkdiag(10,1);

Klqr = lqr(A,B,Q,R);
K = Klqr;

%% PARA EL LINEAL:
pert = [5,2*pi/180,0,0,0]';
x0 = pert;

CCII = [-1,0,.05,0,0];
x0 = CCII;

%% FORMA DE CALCULAR LOS ERROES, o la media:

error = mean(ResponseError(OUT_UAVLinCL_SAT, [1:4]))

%% Vamos a estudir qué le pasa si se come una ráfaga.
% Lo trimamos a la velocidad que toca y todo correcto.
W=2.5;      % Mass
XCG=0.33;   % CG position
V=12;       % Cruise speed
H=100;      % Altitude

X0=[V;0;0;0;H]; U0=[0.6;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% Perturbación de RÁFAGA VERTICAL:
gust = .2 * V;
deltaAoA = atan(gust/V);
x0 = [0,deltaAoA,0,0,0];

% Controlador:
Q = blkdiag(1, 10, 10, 10, 1);
R = blkdiag(10,1);
Klqr = lqr(A,B,Q,R);
K = Klqr;

%%
error = (ResponseError(OUT_UAVLinCL_SAT, [1:4]))
meanerror = mean(ResponseError(OUT_UAVLinCL_SAT, [1:4]))

%%
figure(1)
plot(EXPORT{1}.Values.Time, EXPORT{1}.Values.Data, LineWidth=1)
hold on
plot(EXPORT{2}.Values.Time, EXPORT{2}.Values.Data, LineStyle='--', LineWidth=1)


%% 2. QUE NO FUNCIONE UN CONTROLADOR A UNA VELOCIDAD DE NO SU TRIMADO.
W=2.5;
XCG=0.33;   
V=12;       
H=100;      
X0=[V;0;0;0;H]; U0=[0.6;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);





