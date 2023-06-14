%% 2. ENVOLVENTE DE VUELO: Cálculos y figuras

myblue = [0    0.4470    0.7410];
myred = [0.8500    0.3250    0.0980];
myyellow = [0.9290    0.6940    0.1250];

%% PARÁMETROS:
utils_define_params

%% Límite
% HMAX = 3100;

%% PARTE DE ENTRADA EN PÉRDIDA:
% aoaSTALLdeg = 10;
% vH_STALL = 0:100:HMAX;
% 
% syms v
% ecaoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
% VELS_STALL = 0*vH_STALL;
% for k = 1:length(vH_STALL)
    % disp(k)
    % H = vH_STALL(k);
    % d = density(H);
    % ecaoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
    % VELS_STALL(k) = max(vpasolve(ecaoa == aoaSTALLdeg*pi/180, v));
% end

%% PARTE DE DEFLEXIÓN MÁXIMA:
% vH_DELTA = 0:100:HMAX;
% VELS_DELTA = 0*vH_STALL;
% deltamin = -.5;
% for k = 1:length(vH_DELTA)
%     disp(k)
%     H = vH_DELTA(k);
%     d = density(H);
%     aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
%     ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
%     VELS_DELTA(k) = max(vpasolve(ELEVATOR == deltamin, v));
% end

%% PARTE DE AGUANTAR LA TRACCIÓN IGUALA A LA RESISTENCIA:
% syms v
% vH_TRACC = 0:100:HMAX;
% VELS_TRACC = 0*vH_TRACC;
% for k = 1:length(vH_TRACC)
%     H = vH_TRACC(k);
%     alpha = -CLalpha0/CLalpha + 2*9.81*MASA./(density(H).*v.^2*S*CLalpha);
%     ecTigualD = .5*v^2*S*(CD0 + CDalpha*alpha + CDalpha2*alpha^2) == ...
%     REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
%     VELS_TRACC(k) = max(vpasolve(ecTigualD, v));
% end

%% MONTAMOS LA FIGURA BIEN:
% xisVelocity = [VELS_DELTA, flip(VELS_TRACC)];
% yisHeight = [vH_DELTA, flip(vH_TRACC)];

% fs = 13;
% plot(xisVelocity, yisHeight, LineWidth=1, LineStyle="-")
% xlim([7.5,33.5])
% ylim([0,3100])
% myxlabel('v')
% myylabel('h')
% myplotformat


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GENERATION OF MESH:

dataset = 'A';
dataset = 'B';
dataset = 'S';
dataset = 'XS';

left = 10;
right = 35;

if contains(dataset,'A')
    x = linspace(left, 30, 81);
    y = linspace(100, 3000, 31);

    HMAX = 3100;

    syms v
    vH_DELTA = 0:100:HMAX;
    VELS_DELTA = 0*vH_DELTA;
    vH_TRACC = 0:100:HMAX;
    VELS_TRACC = 0*vH_TRACC;
    deltamin = -.5;
    for k = 1:length(vH_DELTA)
        disp(k)
        H = vH_DELTA(k);
        d = density(H);
        aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
        %
        ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
        VELS_DELTA(k) = max(vpasolve(ELEVATOR == deltamin, v));
        %
        ecTigualD = .5*v^2*S*(CD0 + CDalpha*aoa + CDalpha2*aoa^2) == ...
        REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
        VELS_TRACC(k) = max(vpasolve(ecTigualD, v));
    end

    marker = '.';


elseif contains(dataset,'B')
    x = linspace(10, 30, 21);
    y = linspace(100, 3000, 7);

    HMAX = 3100;

    syms v
    vH_DELTA = 0:100:HMAX;
    VELS_DELTA = 0*vH_DELTA;
    vH_TRACC = 0:100:HMAX;
    VELS_TRACC = 0*vH_TRACC;
    deltamin = -.5;
    for k = 1:length(vH_DELTA)
        disp(k)
        H = vH_DELTA(k);
        d = density(H);
        aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
        %
        ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
        VELS_DELTA(k) = max(vpasolve(ELEVATOR == deltamin, v));
        %
        ecTigualD = .5*v^2*S*(CD0 + CDalpha*aoa + CDalpha2*aoa^2) == ...
        REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
        VELS_TRACC(k) = max(vpasolve(ecTigualD, v));
    end

    marker = 's';

elseif dataset(1) == 'S'    
    x = linspace(10, 35, 6);
    y = linspace(00, 6000, 5);

    HMAX = 6100;

    syms v
    vH_DELTA = 0:100:HMAX;
    VELS_DELTA = 0*vH_DELTA;
    vH_TRACC = 0:100:HMAX;
    VELS_TRACC = 0*vH_TRACC;
    deltamin = -.5;
    for k = 1:length(vH_DELTA)
        disp(k)
        H = vH_DELTA(k);
        d = density(H);
        aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
        %
        ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
        VELS_DELTA(k) = max(vpasolve(ELEVATOR == deltamin, v));
        %
        ecTigualD = .5*v^2*S*(CD0 + CDalpha*aoa + CDalpha2*aoa^2) == ...
        REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
        VELS_TRACC(k) = max(vpasolve(ecTigualD, v));
    end

    marker = 's';

elseif dataset(1) == 'X'
    x = linspace(10, 35, 3);
    y = linspace(0, 6000, 3); 

    HMAX = 6100;

    syms v
    vH_DELTA = 0:100:HMAX;
    VELS_DELTA = 0*vH_DELTA;
    vH_TRACC = 0:100:HMAX;
    VELS_TRACC = 0*vH_TRACC;
    deltamin = -.5;
    for k = 1:length(vH_DELTA)
        disp(k)
        H = vH_DELTA(k);
        d = density(H);
        aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
        %
        ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
        VELS_DELTA(k) = max(vpasolve(ELEVATOR == deltamin, v));
        %
        ecTigualD = .5*v^2*S*(CD0 + CDalpha*aoa + CDalpha2*aoa^2) == ...
        REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
        VELS_TRACC(k) = max(vpasolve(ecTigualD, v));
    end

    marker = 's';

end

% Selección de qué dataset quiero
[X, Y] = meshgrid(x,y);
mask = ones(size(X));
reversemask = ones(size(X))*nan;
% Máscara para cuales pintar:
% llamando 0,1,2,3 a las esquinas de la envolvente:
x0 = 9.28;  y0 = 100;
x1 = 10.72; y1 = 3000;
yl = @(x) (y1-y0)/(x1-x0) * (x-x0) + y0; % y left border 
x2 = 31.69; y2 = 3000;
x3 = 31.75; y3 = 100;
yr = @(x) (y3-y2)/(x3-x2) * (x-x2) + y2; % y right border

for j = 1:size(mask,2)
    for i = 1:size(mask,1)
        if (Y(i,j) > yl(X(i,j))) || (Y(i,j) > yr(X(i,j)))
            mask(i,j) = nan;
            reversemask(i,j) = 1;
        end
    end
end


%%

%%  PLOTS:
xisVelocity = [VELS_DELTA, flip(VELS_TRACC)];
yisHeight = [vH_DELTA, flip(vH_TRACC)];
close all
figure(1)
hold on
% Envolvente
p1 = plot(xisVelocity, yisHeight, LineWidth=2, LineStyle="-", color='c');
p2 = scatter(X, mask.*Y, Marker=marker, LineWidth=1, MarkerEdgeColor='b');
p3 = scatter(X, reversemask.*Y, Marker=marker, LineWidth=1, MarkerEdgeColor='#B0B0B0');

xlim([8,33.5])
if dataset(1) == 'S'; xlim([8,36]); end
if dataset(1) == 'X'; xlim([8,36]); end
ylim([0, HMAX+50])
nfig = 2;
myxlabel('v')
myylabel('h')
title({['\textbf{Dataset ', dataset, '}']}, interpreter='latex', fontsize=18)
hold off
leg = {'Envolvente de vuelo', "Puntos de la malla"};
% leg = {'Envolvente', "Malla"};
legend(leg, Interpreter="latex", fontsize=14)

myplotformat

%%
path = 'C:\Users\ignac\OneDrive - Universidad Politécnica de Madrid\04 - GIA 4 - Cuatri 8\TFG\Presentación\FigsMatlab\';
mysave('fig_dataset_XS.pdf', path)


