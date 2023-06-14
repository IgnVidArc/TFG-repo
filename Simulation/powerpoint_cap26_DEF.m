% powerpoint_cap26_envolvente
% Con las variables del plot de la envolvente:
load("fronteras","-mat")
%%
utils_define_params
% % %xisVelocity
% % %yisHeight
%%
myblue = [0    0.4470    0.7410];
myred = [0.8500    0.3250    0.0980];
myyellow = [0.9290    0.6940    0.1250];

%% GENERATION OF MESH:
dva = 0.25;
dha = 100;

dvb = 1;
dhb = 500;

vmin = 8; vmax = 35;
hmin = 0 ; hmax = 3100;

vva = vmin:dva:vmax;
vvb = vmin:dvb:vmax;

vha = hmin:dha:hmax;
vhb = hmin:dhb:hmax;

[XA, YA] = meshgrid(vva, vha);
[XB, YB] = meshgrid(vvb, vhb);

%% Selección de qué dataset quiero
X = XB;
Y = YB;
mask = ones(size(X));
%% Máscara para cuales pintar:
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
        end
    end
end
%% PLOTS:
xisVelocity = [xleft, flip(xright)];
yisHeight = [yleft, flip(yright)];
figure(1)
hold on
% Envolvente
plot(xisVelocity, yisHeight, LineWidth=1, LineStyle="-")

scatter(X, mask.*Y, Marker=".", LineWidth=1, MarkerEdgeColor=myred)
%%

VELS_DELTA = 0*vha;
VELS_TRACC = 0*vha;
deltamin = -.5;
syms v
for k = 1:length(vha)
    disp(k)
    H = vha(k);
    d = density(H);
    aoa = -CLalpha0/CLalpha + 2*9.81*MASA./(d*v.^2*S*CLalpha);
    ELEVATOR = - Cm0 / Cmde - Cmalpha / Cmde * aoa;
    VELS_DELTA(k) = max(vpasolve(ELEVATOR == deltamin, v)); %%%%%%%%

    alpha = -CLalpha0/CLalpha + 2*9.81*MASA./(density(H).*v.^2*S*CLalpha);
    ecTigualD = .5*v^2*S*(CD0 + CDalpha*alpha + CDalpha2*alpha^2) == ...
    REVmax^2*Dh^4*(CT0 + CTj .* (v/(REVmax*Dh)));
    VELS_TRACC(k) = max(vpasolve(ecTigualD, v));            %%%%%%%%
end



for i = 1:length(vva)
    for j = 1:length(vha)
        v = vva(i);
        h = vva(j);
        if (v > VELS_DELTA(j)) && (v < VELS_TRACC(j))
            scatter(v, h, Marker=".", LineWidth=1, MarkerEdgeColor=myred)
        end
    end
end


xlim([7.5,33.5])
ylim([0,3100])
myxlabel('v')
myylabel('h')
myplotformat
% mysave('cap2_envolvente.pdf')