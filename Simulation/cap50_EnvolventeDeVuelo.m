%% 2. ENVOLVENTE DE VUELO: Cálculos y figuras

myblue = [0    0.4470    0.7410];
myred = [0.8500    0.3250    0.0980];
myyellow = [0.9290    0.6940    0.1250];

%% PARÁMETROS:
H = 100;
M = 2;
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

% Densidad a la altura H

d = density(H);

%% fig:aoa. AOA EQUILIBRIO PARA DISTINTAS VELOCIDADES (MOD)
% Le meto solo una altura pero varios pesos.
V = 6:.5:30;

H = 1000; % H = [1000, 2000, 3000]';
d = density(H);
MASA = [2, 2.5, 3]';
aoa1 = -CLalpha0/CLalpha + 2*9.81*MASA./(d.*V.^2*S*CLalpha);
% H = 2000;
% d = density(H);
% aoa2 = -CLalpha0/CLalpha + 2*9.81*MASA./(d*V.^2*S*CLalpha);
figure(1)
plot(V,aoa1*180/pi, LineWidth=1)
hold on
% hold on
% plot(V,aoa2*180/pi)
xlabel('Velocidad, $V$ [m/s]');
ylabel("\'Angulo de ataque, $\alpha$ [deg]")

% WALL:
Nwall = 40;
xl = [6, 30];
yl = [-10, 40];
xlim(xl)
ylim(yl)

xaux = linspace(xl(1), xl(2), Nwall+1);
stepx = (xl(2) - xl(1)) / Nwall;
stepy = (yl(2) - yl(1)) / Nwall;
xiwall = xaux(1:end-1);
% xdwall = xaux(2:end);
% tita = atan(stepy/stepx);
tita = 70*pi/180;
% l = stepx / cos(tita);
l = stepx/cos(tita) *0.9;
xdwall = xiwall + l * cos(tita);
MATX = [xiwall', xdwall'];
value = 10;
MATalpha(:,1:2) =  [value, value + l * sin(atan(stepy/stepx))];
plot(MATX,  MATalpha, 'k', xl,  [value value], 'k', LineWidth=1)

text(16.75,   value - 1.6, '$\alpha_{max}$', Interpreter='latex', FontSize=14)
nfig=2;
legend({'$M =$ 2.0 kg', '$M =$ 2.5 kg', '$M =$ 3.0 kg'}, Interpreter='latex', FontSize=legfontsize)
myplotformat

name = 'cap2_aoa.pdf';
% mysave(name)


%%
V = linspace(10,30,100);
altitudes = 3000; %altitudes = [1000, 2000, 3000]';
d = density(altitudes);

MASA = [2, 2.5, 3]';

aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*V.^2*S*CLalpha);
ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;

figure(3)
pp = plot(V, ELEVATOR, LineWidth=1);

hold on
% WALL:
Nwall = 40;
xl = [V(1), V(end)];
yl = [-0.5-.1, 0.5 + .1];
xlim(xl)
ylim(yl)

xaux = linspace(xl(1), xl(2), Nwall+1);
stepx = (xl(2) - xl(1)) / Nwall;
stepy = (yl(2) - yl(1)) / Nwall;
xiwall = xaux(1:end-1);
% xdwall = xaux(2:end);
% tita = atan(stepy/stepx);
tita = pi/4;
% l = stepx / cos(tita);
l = stepx;
xdwall = xiwall + l * cos(tita);
MATX = [xiwall', xdwall'];
MATdelta(:,1:2) =  [0.5, 0.5 + l * sin(atan(stepy/stepx))];
plot(MATX,  MATdelta, 'k', xl,  [0.5 0.5], 'k', LineWidth=1)
plot(MATX, -MATdelta, 'k', xl, -[0.5 0.5], 'k', LineWidth=1)

% grid on
xlabel('Velocidad, $V$ [m/s]', Interpreter='latex', FontSize=5)
ylabel("Deflexi\'on del tim\'on, $\delta_e$ [rad]", Interpreter='latex', FontSize=5)
yticks(-.5:.1:.5)

text(19,   0.5 - 0.05, '$\delta_{e,max}$', Interpreter='latex', FontSize=14)
text(19, - 0.5 + 0.05, '$\delta_{e,min}$', Interpreter='latex', FontSize=14)

SetAxisLatex(gca)

% annotation('arrow', 0.1 + [0.2, 0.3], [0.6, 0.4])
legend({'$M =$ 2.0 kg', '$M =$ 2.5 kg', '$M =$ 3.0 kg'}, Interpreter='latex', FontSize=legfontsize)







