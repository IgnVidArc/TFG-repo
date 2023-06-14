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

% ONLY FOLLOWING THE VELOCITY
% Selection of the followed variable:
Ct = [1, 0, 0, 0, 0];
% Augmented matrices:
At = [A, zeros(5,1); -Ct, 0];
Bt = [B; 0 0];
% Cost matrices:
Qt = blkdiag(1, 100, 100, 100, 10, 100); % Augmented state
Rt = blkdiag(100,500);
Kt = lqr(At,Bt,Qt,Rt);

K1 = Kt(:,1:5);
K2 = Kt(:,6);

% FOLLOWING BOTH THE VELOCITY AND ALTITUDE
% Selection of the followed variables:
Ct = [1, 0, 0, 0, 0;
      0, 0, 0, 0, 1];
% Augmented matrices:
At = [A, zeros(5,2); -Ct, zeros(2,2)];
Bt = [B; 0 0; 0 0];
% Cost matrices:
Qt = blkdiag(1, 1000, 1000, 100, 10, 100, 5); % Augmented state
Rt = blkdiag(100,100);
Kt = lqr(At,Bt,Qt,Rt);

K1 = Kt(:,1:5);
K2 = Kt(:,6:7);
