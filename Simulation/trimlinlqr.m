%% trimlinlqr.m
% Parameters:
W = 2.5;	% Mass [kg]
XCG = 0.33;	% Position of center of gravity [-]
IP = 0.1568;% Moment of inertia [kg m^2]
% Trim conditions:
V = 15;		% Velocity [m/s]
H = 1000;	% Altitude [m]
% Trim:
X0 = [V;0;0;0;H];
U0 = [0.5;-0.1];
Y0 = [];
IX = [1,4,5];
IU = [];
IY = [];
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
% Linear Model:
[A,B,C,D] = linmod('UAVTrimh',Xtrim,Utrim);
% Cost matrices:
Q = blkdiag(1,100,100,100,10);
R = blkdiag(100,500);
% Controller:
K = lqr(A,B,Q,R);
