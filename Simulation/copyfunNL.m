function XD = copyfunNL(XV,UV,param)

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

VT = XV(1);	    % Velocidad aerodinámica
ALPHA = XV(2);	% Angulo de ataque
THETA = XV(3);  % Angulo de asiento
Q = XV(4);	    % Velocidad angular de cabeceo
H = XV(5);      % Altura

% Variables de control

Dt = UV(1);	    % Acelerador
De = UV(2);	    % Deflexión del timón de profundidad

% Parameters

MASA = param(1);    % Masa
XCG = param(2);     % Posición del centro de gravedad (adim, xcg/c)

% Densidad a la altura H

d = density(H);

% Algunos calculos previos

U = VT*cos(ALPHA);	    % Componentes de la velocidad en ejes cuerpo
W = VT*sin(ALPHA);	

PDS = 0.5*d*(VT^2)*S;   % (Presión dinámica) * (Superficie alar)

% Parametros Grupo Motopropulsor

Dh = 0.254;             % Diametro de la helice (m)
REVmax = 222;           % Revoluciones máximas del motor (rev/s)

CT0 = 0.13805;          % Coeficientes de traccion
CTj = -0.2049;
REV = REVmax*Dt;        % Revoluciones instantáneas
J = VT/(REV*Dh);        % Coeficiente de velocidad
C_T = CT0+CTj*J;        % Coef de tracción
T = C_T*d*REV^2*Dh^4;   % Traccion

% Coeficientes adimensionales de fuerzas y momentos (ejes viento)

CLT = CLalpha0 + CLalpha*ALPHA + CLq*Q*C*0.5/VT;         
CDT = CD0 + CDalpha*ALPHA + CDalpha2*ALPHA^2;  
CZ  = - CDT*sin(ALPHA) - CLT*cos(ALPHA); % Cz en ejes cuerpo para añadir despues el efecto del centro de gravedad
CmT = Cm0 + (Cmalpha*ALPHA) + (Cmde*De) + (CZ*(XCG-XCGR)) + (C*0.5*Cmq*Q/VT); % Falta la componente alphadot,la añadiremos después

% Fuerzas aerodinámicas

L = CLT*PDS;
D = CDT*PDS;

% Sistema de ecuaciones diferenciales de la dinámica del vuelo

UDOT = -Q*W + (T - MASA*G*sin(THETA) - D*cos(ALPHA) + L*sin(ALPHA))/MASA;
WDOT = Q*U + (MASA*G*cos(THETA) - D*sin(ALPHA) - L*cos(ALPHA))/MASA;
ALPHADOT = (U*WDOT - W*UDOT)/VT^2;
CmT = CmT + Cmalphadot*C/(2*VT)*ALPHADOT; % Completamos Cm
QDOT = CmT*PDS*C/IP;

VTDOT = (U*UDOT + W*WDOT)/VT;

% Vector con las derivadas de las variables de estado

XD = zeros(5,1);
XD(1) = VTDOT;
XD(2) = ALPHADOT;
XD(3) = Q;
XD(4) = QDOT;
XD(5) = VT*sin(THETA-ALPHA);
