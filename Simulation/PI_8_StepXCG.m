%% Comparing the step response with various XCG
% Additional parameters:
W = 2.5;
XCG = 0.33;
% Trim conditions
V = 15;
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh_modXCG',X0,U0,Y0,IX,IU,IY);
[A,B,C,D]=linmod('UAVTrimh_modXCG',Xtrim,Utrim);

% LQR Controller:
% Q = blkdiag(1, 1000, 1000, 100, 1);
% R = blkdiag(100, 100);
% [K,S,P] = lqr(A,B,Q,R);

%%
cosa = ss(A,B,C,D);
eig_mod = eig(A)
[estep_mod, time_mod] = step(cosa);
figure
plot(time_mod, estep_mod(:,1,2))
hold on

%%
cosa = ss(A,B,C,D);
eig_original = eig(A)
[estep_original , time_orginial] = step(cosa);
% figure
%%
plot(time_orginial, estep_original(:,1,2))

%%
vXCG = [.29, .33, .40];
eigs = zeros(5,length(vXCG));
for k = 1:length(vXCG)
    XCG = vXCG(k);
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh_modXCG',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh_modXCG',Xtrim,Utrim);
    eigs(:,k) = eig(A);
end
eigs
%%
% Si simulas el UAVLinOL sin el step, ves que los modos propios son
% parecidísimos. Y que cuesta ver las diferencias.
% Nonetheless, sí ponerlo en el TFG, cuando se presententen los modos
% autovalores.


%%
% Probamos ahora a meterle un step en timón para ver el pico y esas cosas.
x0 = [0,0,0,0,0]';
StopTime = 50;
sim('UAVLinOL_step')
SIMDATA(length(vXCG),1) = outY;

%%
StopTime = 50;
vXCG = [.25, .33, .40];
x0 = [1,0,0,0,0]' * 0;
eigs = zeros(5,length(vXCG));
SIMDATA(length(vXCG),1) = outY;
% First index for vXCG case
% Second index: 1=time, 2=outY
for k = 1:length(vXCG)
    disp(k)
    XCG = vXCG(k);
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh_modXCG',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh_modXCG',Xtrim,Utrim);
    sim('UAVLinOL_step');
    SIMDATA(k) = outY;
end
%%
% State to plot:
comp = [1];
figure(10)
hold on
leg = cell(3,1);
for k=1:length(vXCG)
    leg{k} = ['$\hat{x}_{cg} = ' num2str(vXCG(k)) '$'];
    plot(SIMDATA(k).time, SIMDATA(k).signals.values(:,comp), LineWidth=1)
end
myxlabel('t')
myylabel('v')
legend(leg, Interpreter='latex', FontSize=12)
myplotformat
% xlim([0,12])

%%
% figure(10000)
which = 1;
time_ = SIMDATA(which).time;
V_ = SIMDATA(which).signals.values(:,1);
alpha_ = SIMDATA(which).signals.values(:,2);
theta_ = SIMDATA(which).signals.values(:,3);
q_ = SIMDATA(which).signals.values(:,4);
h_ = SIMDATA(which).signals.values(:,5);

gamma_ = theta_ - alpha_;
V_horiz_ = V_ .* cos(gamma_);
xx = zeros(length(time_),1);
for k = 2:length(time_)
    xx(k) = xx(k-1) + V_horiz_(k) * (time_(k) - time_(k-1));
end
% x_horiz_ = V_horiz_ .* time_;
plot(xx, h_)
axis equal
% xhoriz = 
% plot(SIMDATA(1).signals.values(1:50,1)-SIMDATA(2).signals.values(1:50,1))