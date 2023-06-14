% THIS IS TO SEE THE PERFORMANCE OF A CONTROLLER TRIMMED AROUND SOME
% CONDITION, IN EVERY OTHER CONDITION.

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
[A,B,C,D] = linmod('UAVTrimh',Xtrim,Utrim);


%%


% LQR Controller:
Q = blkdiag(1, 100, 100, 100, 10);
R = blkdiag(100, 100);
K = lqr(A,B,Q,R);


StopTime = 5;
x0 = [-1, .0, .05, 0.1, -1.5];
out = sim("UAVLinCL_DEF_v0");

nfig = 2;
getxplot
% mysave('cap3_lqr_100_100_x.pdf')
getuplot
% xlim([0,.15])
% mysave('cap3_lqr_100_100_u.pdf')

%% Detalle de derivada inicial:
% Q = blkdiag(1, 1000, 1000, 100, 1);
% R = blkdiag(100, 100);
% [K,S,P] = lqr(A,B,Q,R);
% x0 = [-2, .05, .1, 2, -2];
% out = sim("UAVLinCL_DEF_v0");
% 
% nfig = 2;
% getxplot
% % mysave('cap3_lqr_100_100_x.pdf')
% getuplot
% xlim([0,.1])
% % mysave('cap3_lqr_100_100_u.pdf')



%%
Q = blkdiag(1, 100, 100, 100, 10);
R = blkdiag(100, 500);
[K,S,P] = lqr(A,B,Q,R);
x0 = [-1, .0, .05, 0.1, -1.5];
out = sim("UAVLinCL_DEF_v0");

nfig = 2;
getxplot
% mysave('cap3_lqr_100_500_x.pdf')
getuplot
% xlim([0,.15])
% xlim([0,.1])
% mysave('cap3_lqr_100_500_u.pdf')

%%
PI = PerformanceIndex(outX, outU, Q, R);






