%% 2. ENVOLVENTE DE VUELO: Cálculos y figuras

myblue = [0    0.4470    0.7410];
myred = [0.8500    0.3250    0.0980];
myyellow = [0.9290    0.6940    0.1250];

%% PARÁMETROS:
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

% Densidad a la altura H

d = density(H);

%% fig:aoa. AOA EQUILIBRIO PARA DISTINTAS VELOCIDADES
V = 6:.5:30;

H = [1000, 2000, 3000]';
d = density(H);
aoa1 = -CLalpha0/CLalpha + 2*9.81*MASA./(d.*V.^2*S*CLalpha);
% H = 2000;
% d = density(H);
% aoa2 = -CLalpha0/CLalpha + 2*9.81*MASA./(d*V.^2*S*CLalpha);
figure(1)
plot(V, aoa1*180/pi, LineWidth=1)
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

legend({'$H =$ 1000 m', '$H =$ 2000 m', '$H =$ 3000 m'}, Interpreter='latex', FontSize=12)
nfig=1;
myplotformat

name = 'cap2_aoa.pdf';
% mysave(name)

%% REYNOLDS

nu = 1.252E-5;   % (m^2/s)
Re = C*V/nu;
plot(V,Re)
% En todos los casos estamos del orden de 10^5 --> para meternos en
% http://airfoiltools.com/airfoil/details?airfoil=n0012-il
% y sacar que entra en pérdida entorno a los 15 grados.


%% ENVOLVENTE DE VUELO: PRIMERAS DOS FRONTERAS
HMAX = 3000;

% PARTE DE ENTRADA EN PÉRDIDA:
aoaSTALLdeg = 10;
vH_STALL = 100:100:HMAX;

syms v
ecaoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
VELS_STALL = 0*vH_STALL;
for k = 1:length(vH_STALL)
    disp(k)
    H = vH_STALL(k);
    d = density(H);
    ecaoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
    VELS_STALL(k) = max(vpasolve(ecaoa == aoaSTALLdeg*pi/180, v));
end


% PARTE DE DEFLEXIÓN MÁXIMA:
vH_DELTA = 100:100:HMAX;
VELS_DELTA = 0*vH_STALL;
deltamin = -.5;
for k = 1:length(vH_DELTA)
    disp(k)
    H = vH_DELTA(k);
    d = density(H);
    aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
    ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
    VELS_DELTA(k) = max(vpasolve(ELEVATOR == deltamin, v));
end
%% fig:fronteras
figure
plot(VELS_STALL,vH_STALL, LineWidth=1)
hold on
plot(VELS_DELTA, vH_DELTA, LineWidth=1)
% xlim([5,35])
xlabel('Velocidad, $V$ (m/s)')
ylabel('Altura, $H$ (m)')
l1 = "\'Angulo de entrada en p\'erdida";
l2 = "Deflexi\'on m\'inima del tim\'on";
legend({l1, l2}, Interpreter='latex', FontSize=12)
myplotformat
name = 'cap2_fronteras.pdf';
% mysave(name)

%% solo mini cálculo de comporbación T=D:
H = 100;
alpha = @(v) -CLalpha0/CLalpha + 2*9.81*MASA./(density(H).*v.^2*S*CLalpha);
DRAG = @(v, alpha) .5*v^2*S*(CD0 + CDalpha*alpha + CDalpha2*alpha^2);
TRACC = @(v) REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
vel = 30.0;
aoa = alpha(vel);
drag = DRAG(vel, aoa)
tracc = TRACC(vel)

%% PARTE DE AGUANTAR LA TRACCIÓN IGUALA A LA RESISTENCIA:
syms v
vH_TRACC = 100:100:HMAX;
VELS_TRACC = 0*vH_STALL;
for k = 1:length(vH_TRACC)
    H = vH_TRACC(k);
    alpha = -CLalpha0/CLalpha + 2*9.81*MASA./(density(H).*v.^2*S*CLalpha);
    ecTigualD = .5*v^2*S*(CD0 + CDalpha*alpha + CDalpha2*alpha^2) == ...
    REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
    VELS_TRACC(k) = max(vpasolve(ecTigualD, v));
end


plot(VELS_TRACC, vH_TRACC, LineWidth=1)
myxlabel('v')
myylabel('h')
xlim([31.70, 31.75])
myplotformat
% mysave('cap2_fronteraderecha.pdf')

%% PLOT CURVA EMPUJE Y CURVA DE RESISTENCIA
% Empuje vs velocidad para varias REVOLUCIONES:
v = linspace(10,40,100);
H = [1000]';
d = density(H);
deltas = [0.3:0.1:1]';
Ns = deltas * REVmax;
aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d.*v.^2*S*CLalpha);
DRAG = .5*d.*v.^2*S.*(CD0 + CDalpha*aoa + CDalpha2*aoa.^2);
TRACC = d.*Ns.^2*Dh^4.*(CT0 + CTj .* (v./(Ns*Dh)));
figure(3)
% plot(v, DRAG)
hold on
plot(v, TRACC)
myxlabel('v')
myylabel('h')
myplotformat

%% AHORA SÍ, EMPUJE Y TRACCIÓN VS VELOCIDAD, PARA Nmax
v = linspace(10,40,100);
H = [1000:1000:3000]';
d = density(H);
aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d.*v.^2*S*CLalpha);
DRAG = .5*d.*v.^2*S.*(CD0 + CDalpha*aoa + CDalpha2*aoa.^2);
TRACC = d.*REVmax.^2*Dh^4.*(CT0 + CTj .* (v./(REVmax*Dh)));
figure(4)
p(1) = plot(v, DRAG(1,:), LineWidth=1, Color=myblue);
hold on
p(2) = plot(v, DRAG(2,:), LineWidth=1, Color=myred);
p(3) = plot(v, DRAG(3,:), LineWidth=1, Color=myyellow);
plot(v, TRACC(1,:), LineWidth=1, Color=myblue, LineStyle='--')
plot(v, TRACC(2,:), LineWidth=1, Color=myred, LineStyle='--')
plot(v, TRACC(3,:), LineWidth=1, Color=myyellow, LineStyle='--')
myxlabel('v')
ylabel('Fuerzas, $D$ y $T$ [N]')
legend({'$H =$ 1000 m', '$H =$ 2000 m', '$H =$ 3000 m', ...
        '$H =$ 1000 m', '$H =$ 2000 m', '$H =$ 3000 m'}, Interpreter='latex', FontSize=12)
myplotformat
% mysave('cap2_TyD.pdf')


% % PARTE DE VUELO EQUILIBRADO:
% syms v
% vH_EQUIL = 100:100:HMAX;
% VELS_EQUIL = 0*vH_STALL;
% for k = 1:length(vH_TRACC)
%     H = vH_TRACC(k);
%     alpha = -CLalpha0/CLalpha + 2*9.81*MASA./(density(H).*v.^2*S*CLalpha);
%     ecEQUIL = - Cm0 / Cmde - Cmalpha / Cmde * alpha == -0.5;
%     VELS_EQUIL(k) = max(vpasolve(ecEQUIL, v));
% end
% plot(VELS_EQUIL, vH_EQUIL)
% hold off


%% TEST: POTENICAS ÚTILES Y DE RESISTENCIA
% syms v
% vH_POT = 100:100:HMAX;
% VELS_POT = 0*vH_STALL;
% for k = 1:length(vH_POT)
%     H = vH_POT(k);
%     alpha = -CLalpha0/CLalpha + 2*9.81*MASA./(density(H).*v.^2*S*CLalpha);
%     ecTigualD = v*.5*v^2*S*(CD0 + CDalpha*alpha + CDalpha2*alpha^2) == ...
%     v*REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
%     VELS_POT(k) = max(vpasolve(ecTigualD, v));
% end
% 
% 
% plot(VELS_POT, vH_POT, LineWidth=1)
% myxlabel('v')
% myylabel('h')
% xlim([31.70, 31.75])
% myplotformat

v = linspace(10,40,50);
H = [1000:1000:10000]';
% H = 100;
d = density(H);
aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d.*v.^2*S*CLalpha);
PD = v.*.5.*d.*v.^2*S.*(CD0 + CDalpha*aoa + CDalpha2*aoa.^2);
PU = v.*d.*REVmax.^2*Dh^4.*(CT0 + CTj .* (v./(REVmax*Dh)));
figure(6)
plot(v, PD(:,:), LineWidth=1, Color=myblue);
hold on
% p(2) = plot(v, PD(2,:), LineWidth=1, Color=myred);
% p(3) = plot(v, PD(3,:), LineWidth=1, Color=myyellow);
plot(v, PU(:,:), LineWidth=1, Color=myblue, LineStyle='--')
% plot(v, PU(2,:), LineWidth=1, Color=myred, LineStyle='--')
% plot(v, PU(3,:), LineWidth=1, Color=myyellow, LineStyle='--')
myxlabel('v')
ylabel('Potencias, $P_d$ y $P_u$ [W]')
% legend({'$H =$ 1000 m', '$H =$ 2000 m', '$H =$ 3000 m', ...
%         '$H =$ 1000 m', '$H =$ 2000 m', '$H =$ 3000 m'}, Interpreter='latex', FontSize=12)
myplotformat


%% MONTAMOS LA FIGURA BIEN:
xisVelocity = [VELS_DELTA, flip(VELS_TRACC)];
yisHeight = [vH_DELTA, flip(vH_TRACC)];

fs = 13;
plot(xisVelocity, yisHeight, LineWidth=1, LineStyle="-")
xlim([7.5,33.5])
ylim([0,3100])
myxlabel('v')
myylabel('h')
myplotformat
% mysave('cap2_envolvente.pdf')



%% Curva de tracción de la hélice
syms n
H = 100;
V = 10;
d = density(H);
Tracc = d .* n^2*Dh^4 .* (CT0 + CTj.*(V./(n*d)));
fplot(Tracc)
xlim([0,222])

% Tiene un mínimo ridículo en torno a muy pocas rpm y luego solo crece.
% Hence, envuelta será con REV maximas.

%% Vuelo equilibrado:
V = linspace(10,30,50);
d = density(500);
aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*V.^2*S*CLalpha);

ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
figure(1)
plot(aoa,ELEVATOR)
%%
figure(2)
plot(V, ELEVATOR)
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
xdwall = xiwall + stepx * cos(atan(stepy/stepx));
MATX = [xiwall', xdwall'];
MATdelta(:,1:2) =  [0.5, 0.5 + stepx * sin(atan(stepy/stepx))];
MATdown(:,1:2) = - [0.5, 0.5 + stepx * sin(atan(stepy/stepx))];
figure
plot(MATX,  MATdelta, 'k', xl,  [0.5 0.5], 'k')
ylim([.4, .6])
% plot(MATX, -MATdelta, 'k', xl, -[0.5 0.5], 'k')

%%
V = linspace(10,30,100);
altitudes = [1000, 2000, 3000]';
d = density(altitudes);
aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*V.^2*S*CLalpha);
ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;

figure(3)
pp = plot(V, ELEVATOR, LineWidth=1);
%%
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
legend({'$H =$ 1000 m', '$H =$ 2000 m', '$H =$ 3000 m'}, Interpreter='latex', FontSize=12)







