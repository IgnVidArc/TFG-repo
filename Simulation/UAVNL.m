function XD = UAVNL(XV,UV,param)
% Empty output vector:
XD = zeros(5,1);

% UAV PARAMETERS
S = 0.4121;               % Wing surface [m^2]
C = 0.2192;               % Mean aerodynamic chord [m]
XCGR = 0.33;              % Reference center of gravity position [-]
g = 9.8;                  % Gravitation acceleration [m/s^2]

% Stability and control derivatives:
CL0 = 0.405;    
CLalpha = 5.379292;
CLq = 11.133615;
CD0 = 0.0277;
CDalpha = 0.14897;
CDalpha2 = 0.9848;
Cm0 = 0.0418;     
Cmalpha = -6.0;
Cmalphadot = -7.0;
Cmde = -1.552;
Cmq = -18.422237;

% State variables:
VT    = XV(1);	        % Speed [m/s]
ALPHA = XV(2);	        % Angle of atack [rad]
THETA = XV(3);          % Pitch angle [rad]
Q     = XV(4);	        % Pitch angular velocity [rad/s]
H     = XV(5);          % Height [m]

% Control variables:
Dt = UV(1);	            % Thrust position [-]
De = UV(2);	            % Elevator position [rad]

% Aditional parameters:
M   = param(1);
XCG = param(2);
IP  = param(3);

% Modified elevator control coefficient:
deltaXCG = XCG - XCGR;
Cmde = Cmde * (.80 - deltaXCG*C)/.80;

% Density at altitude H:
d = density(H);

% Previous calculations
U = VT*cos(ALPHA);      % X-body velocity component
W = VT*sin(ALPHA);	    % Z-body velocity component
PDS = 0.5*d*(VT^2)*S;	% Dynamic pressure * Wind surface [Pa]

% Proupulsion parameters:
Dh = 0.254;             % Propeller diameter [m]
REVmax = 222;           % Max revolutions [rev/s]
CT0 = 0.13805;          % Traction coefficients
CTj = -0.2049;
REV = REVmax*Dt;        % Instantaneous revolutions [rev/s]
J = VT/(REV*Dh);        % J coeffient [-]
C_T = CT0 + CTj*J;      % Traction coefficient [-]
T = C_T*d*REV^2*Dh^4;   % Traction [N]

% Forces and moment coefficients:
CLT = CL0 + CLalpha*ALPHA + CLq*Q*C*0.5/VT;         
CDT = CD0 + CDalpha*ALPHA + CDalpha2*ALPHA^2;  
CZ = -CDT*sin(ALPHA) - CLT*cos(ALPHA);
CmT = Cm0 + (Cmalpha*ALPHA) + (Cmde*De) + (CZ*(XCG-XCGR)) ...
    + (C*0.5*Cmq*Q/VT);

% Aerodynamic forces:
L = CLT*PDS;
D = CDT*PDS;

% System of differential equations:
UDOT = -Q*W + (T - M*g*sin(THETA) - D*cos(ALPHA) + L*sin(ALPHA))/M;
WDOT = Q*U + (M*g*cos(THETA) - D*sin(ALPHA) - L*cos(ALPHA))/M;
ALPHADOT = (U*WDOT - W*UDOT)/VT^2;
CmT = CmT + Cmalphadot*C/(2*VT)*ALPHADOT;
QDOT = CmT*PDS*C/IP;
VTDOT = (U*UDOT + W*WDOT)/VT;

% Output vector:
XD(1) = VTDOT;
XD(2) = ALPHADOT;
XD(3) = Q;
XD(4) = QDOT;
XD(5) = VT*sin(THETA-ALPHA);
