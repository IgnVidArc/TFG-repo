%% ESTE YA NO ES CON EL MODELO LINEAL, CLARO

% Ejecutas PI_6_Tracker_bigGuy con pendientes [0, rampH]:

%%

% leeeeeeeeeemos el logsout que genera UAVNLCL_TRACK_GS


%%
close all
nfig=2;
if nfig==1
    legfontsize = 12;
elseif nfig==2
    legfontsize = 15;
else
end
time = logsout{1}.Values.Time;
%% PARA PLOT VELOCIDAD Y r(t)
figure(1)
plot(time, logsout{1}.Values.Data(:,1), linewidth=1)
hold on
plot(time, logsout{4}.Values.Data(:,1), linewidth=1)
myxlabel('t')
myylabel('vabs')
mylegend('vyref', 'northwest')
% legend({'V', '$r_v(t)$'}, Interpreter="latex", FontSize=legfontsize)
myplotformat
% mysave('cap3_trackBoth_a.pdf')

%% ESTADO INCREMENTAL PARA 2,3,4.
figure(2)
plot(time, logsout{5}.Values.Data(:,2:4), linewidth=1)
% xlim([0,35])
nfig = 2;
myxlabel('t')
myylabel('allx')
% ylabel('Velocidad absoluta [m/s]', Interpreter='latex')
% legend({'V', '$r(t)$'}, Interpreter="latex", FontSize=12)
mylegend('234')
myplotformat
% mysave('cap3_trackBoth_b.pdf')

%% ALTURA ABSOLUTA
figure(3)
plot(time, logsout{1}.Values.Data(:,5), linewidth=1)
hold on
plot(time, logsout{4}.Values.Data(:,2), linewidth=1)
% xlim([0,35])
nfig = 2;
myxlabel('t')
myylabel('habs')
mylegend('hyref', 'northwest')
myplotformat
% mysave('cap3_trackBoth_c.pdf')

%% CONTROL ABSOLUTO
figure(4)
plot(time, logsout{3}.Values.Data(:,:), linewidth=1)
% xlim([0,35])
nfig = 2;
myxlabel('t')
myylabel('uabs')
mylegend('deltas')
myplotformat
% mysave('cap3_trackBoth_d.pdf')
