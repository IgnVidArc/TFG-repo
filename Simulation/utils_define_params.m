%% PARAMS
H = 100;
M = 2.5;
XCG = .33;
param = [M, XCG];

% DATOS DE LA AERONAVE

S = 0.4121;                 % Superficie alar (m^2)
C = 0.2192;                 % Cuerda media aerodinámica (m)
% MASA = 2.5;                 % Masa (Kg)
XCGR = 0.33;                % Posición de referencia del CG (xcg/c)
IP = 0.156794;              % Momento de inercia (Kg*m^2)
% d = 1.225;                  % Densidad (Kg/m^3)
G = 9.8;                    % Gravedad (m/s^2)
% XCG = .33;                  % CG no coincide con el de referencia

% Derivadas de estabilidad y control adimensionales

CLalpha0 = 0.405;    
CLalpha = 5.379292;
CLq = 11.133615;
CD0 = 0.0277;
CDalpha = 0.14897;
CDalpha2 = 0.9848;
Cm0 = 0.0418;     
Cmalpha = -6.0; %-1.812981 más inestable en cabeceo
Cmalphadot = -7.0;
Cmde = -1.552;
Cmq = -18.422237;

% Variables de estado

% VT = XV(1);	    % Velocidad aerodinámica
% ALPHA = XV(2);	% Angulo de ataque
% THETA = XV(3);  % Angulo de asiento
% Q = XV(4);	    % Velocidad angular de cabeceo
% H = XV(5);      % Altura

Dh = 0.254;             % Diametro de la helice (m)
REVmax = 222;           % Revoluciones máximas del motor (rev/s)
CT0 = 0.13805;          % Coeficientes de traccion
CTj = -0.2049;
% REV = REVmax*Dt;        % Revoluciones instantáneas
% J = VT/(REV*Dh);        % Coeficiente de velocidad
% C_T = CT0+CTj*J;        % Coef de tracción

% Variables de control

% Dt = UV(1);	    % Acelerador
% De = UV(2);	    % Deflexión del timón de profundidad

% Parameters

MASA = param(1);    % Masa
XCG = param(2);     % Posición del centro de gravedad (adim, xcg/c)

d = density(H);