%% RECOVERING PREDICTED CONTROLLER FROM csv FILE FROM PYTHON AND SEEING ITS PERFORMANCE

%%%%%%%%%%%%%%%%%%%%%% OBSOLOTE %%%%%%%%%%%%%%%%%%%%%%
% Additional parameters:
W = 2.5;
XCG = 0.33;

% ----- TO COPY FROM PYTHON: ------
trim_conditions = [18.99, 1492];
controllername = 'SingleController_002_test2_checkgood-18.99-1492.csv';


% GETTING LINEAR MODEL IN THAT POINT
V = trim_conditions(1);  
H = trim_conditions(2);
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% GETTING THE EXPORTED CONTROLLER FROM FILE:
path = '../Controllers/';
Kpredicted = readmatrix([path controllername]);


%% 
x0 = [-1, 0, -.5, -.5, 0]';
StopTime = 15;
Q = blkdiag(1, 1000, 1000, 100, 1);
R = blkdiag(1, 1) * 100;
%% SEEING DIFERENCES:

K = Kpredicted;
sim("UAVLinCL_DEF_v0");
outXpredicted = outX;
outUpredicted = outU;
K = lqr(A, B, Q, R);
sim("UAVLinCL_DEF_v0");
outXnew = outX;
outUnew = outU;

QuickCompareSims([outXnew, outXpredicted], 1:5)
QuickCompareSims([outUnew, outUpredicted], 1:2)


%% Plotting the response:
Show
f = gcf;
% f.Position = [10, 200, 1000, 420];

%% PERFROMANCE INDEX:
Q0 = blkdiag(1, 100, 100, 0, 0);
R0 = blkdiag(1, 1) * 1;
PI = PerformanceIndex(outX, outU, Q0, R0);


%% COMPARING THE RESPONSE TO ANOTHER CRONTROLLER DESIGNED THERE:
% FLIGHT CONDITIONS
trim_conditions = [14.92, 1492];
V = trim_conditions(1); H = trim_conditions(2);
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
Q = blkdiag(1, 1000, 1000, 100, 1);
R = blkdiag(1, 1);
[K,S,P] = lqr(A,B,Q,R);
K
%%
x0 = [-1, 0, -.5, -.5, 0]';
StopTime = 5;
sim("UAVLinCL_DEF_v0");
% Show

%% COMPARE THESE TWO RESULTS
compsX = 1:5;
compsU = 1:2;
QuickCompareSims([outXpredict, outX], compsX)

%%
% this is after having the state2k function already implemented in matlab
% this is after having the state2k function already implemented in matlab
% this is after having the state2k function already implemented in matlab
% this is after having the state2k function already implemented in matlab
state = [14.92, 1492];
tic
K = state2k(state);
toc
%%
outXpredict = outX;



