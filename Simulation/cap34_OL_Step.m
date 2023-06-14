%% 
% Additional parameters:
W = 2.5;
XCG = 0.33;
% Trim conditions
V = 15;
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

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
mysave('cap3_OL_step.pdf')
