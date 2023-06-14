%% 
% Additional parameters:
W = 2.5;
XCG = 0.33;
IP = 0.1568;% Moment of inertia [kg m^2]
% Trim conditions
V = 15;
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh5D',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh5D',Xtrim,Utrim);

x0 = [1, 0, 0, 0, 0]';
%%
StopTime = 50;
out = sim('UAVLinOL');
%%
figure(1)
plot(outY.time, outY.signals.values(:,[1,5]), LineWidth=1)
myxlabel('t')
% ylabel('X(1) [m/s], X(5) [m]')
% legend({'$V$', '$H$'}, Interpreter='latex', FontSize=12)
nfig = 2;
myylabel('x')
mylegend('15')
myplotformat
% mysave('cap3_ol_15.pdf')
%%
figure(2)
plot(outY.time, outY.signals.values(:,2:4), LineWidth=1)
myxlabel('t')
nfig = 2;
myylabel('x')
mylegend('234')

myplotformat
% mysave('cap3_ol_234.pdf')
%%
myred = [0.8500    0.3250    0.0980];
figure(3)
semilogx(outY.time, outY.signals.values(:,5), LineWidth=1, Color=myred)
myxlabel('t')
myylabel('h')
mylegend('h')
xlim([10^-1, 10^5])
xticks(10.^[-1, 0, 1, 2, 3, 4, 5])
myplotformat
grid on
grid minor
ax = gca;
ax.YGrid = 'off';
% mysave('cap3_ol_log.pdf')
