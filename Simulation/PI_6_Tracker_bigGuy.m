%% 

% GETTING THE DISCRETE CONTROLLERS FIRST:
W = 2.5; XCG = 0.33;

% Cost things -------------
Q_gen = blkdiag(1, 100, 100, 100, 10);
R_gen = blkdiag(100, 500);


% Discrete things: -------------
    Vs_range = [[10; 15], [15;20], [20;25], [25;30]];
    Vs_trim_range = mean(Vs_range);

    Vs_range = [10; 45];
    Vs_trim_range = 10;     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Hs_range = zeros(size(Vs_range));
Hs_range(:,:) = 1000;
Hs_trim_range = Hs_range(1,:);
Ks_range = get_Ks_range(Vs_range, Vs_trim_range, Hs_range, Hs_trim_range, Q_gen, R_gen);

% RAMP THNIGS:
path_angle = 7;
rampH = Vs_trim_range*tan(path_angle*pi/180);
rawslopes = [1, rampH];

%% THE OTHER PART OF THE MODEL:
% Additional parameters:
W = 2.5; XCG = 0.33;
% Trim conditions:
% V = 20;
V = Vs_trim_range;
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim and linear model:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% LQR Controller:
[K,S,P] = lqr(A,B,Q_gen,R_gen);

x0 = zeros(5,1);

% Este script con lo que funciona es con el UAVNLCL_TRACK_NN.slx.

% TRAKING:
q_xi_V = 100;
q_xi_H = 5;

caso = 2;
    % 1 --> solo tracking de la velociadad
    % 2 --> tracking de both V y H.
    % 3 --> solo altura
h0t = 0;
if caso == 1
    slopes = [rawslopes(1), 0];
    % Following the velocity
    % Selection of the followed variable:
    Ct = [1, 0, 0, 0, 0];
    % Matrices ampliadas del sistema para el tracking:
    At = [A, zeros(5,1); -Ct, 0];
    Bt = [B; 0 0];
    % Matrices de costes:
    % % % Qt = blkdiag(1, 1000, 1000, 100, 1, 1000);
    % % % Rt = blkdiag(100,5000);
    Qt = [Q_gen, zeros(5,1); zeros(1,5), q_xi_V];
    Rt = R_gen;
    Kt = lqr(At,Bt,Qt,Rt);
    
    K1 = Kt(:,1:5);
    K2 = Kt(:,6);

elseif caso == 2
    slopes = rawslopes;
    % Following both the VELOCITY AND HEIGHT.
    % Selection of the followed variable:
    Ct = [1, 0, 0, 0, 0;
          0, 0, 0, 0, 1];
    % Matrices ampliadas del sistema para el tracking:
    At = [A, zeros(5,2);
          -Ct, zeros(2,2)];
    Bt = [B;
          0 0;
          0 0];
    % Matriz de los costes de estado:
    % % % Qt = blkdiag(1, 1000, 1000, 100, 1, 100, 100);      
    % % % Rt = blkdiag(100,5000);
    
    Qt = [Q_gen, zeros(5,2); zeros(2,5), blkdiag(q_xi_V, q_xi_H)];
    Rt = R_gen;
    Kt = lqr(At,Bt,Qt,Rt);
    
    K1 = Kt(:,1:5);
    K2 = Kt(:,6:7);

elseif caso == 3
    h0t = 1;
    slopes = [0, rawslopes(2)];
    % Following only HEIGHT.
    % Selection of the followed variable:
    Ct = [0, 0, 0, 0, 1];
    % Matrices ampliadas del sistema para el tracking:
    At = [A, zeros(5,1);
          -Ct, zeros(1,1)];
    Bt = [B;
          0 0];
    % Matriz de los costes de estado:
    % % % Qt = blkdiag(1, 1000, 1000, 100, 1, 100, 100);      
    % % % Rt = blkdiag(100,5000);
    
    Qt = [Q_gen, zeros(5,1); zeros(1,5), q_xi_H];
    Rt = R_gen;
    Kt = lqr(At,Bt,Qt,Rt);
    
    K1 = Kt(:,1:5);
    K2 = Kt(:,6);
end

%%
% Initial condition for the tracking problem:
x0t = zeros(5,1);
x0t(5) = h0t;

out = sim('UAVNLCL_TRACK_GS.slx');


%%


function Ks_range = get_Ks_range(Vs_range, Vs_trim_range,  Hs_range, Hs_trim_range, Q, R)

    Ks_range = zeros(2,5*size(Vs_range,2));

    W = 2.5; XCG = 0.33;
  
    U0=[0.5;-0.1]; Y0=[];
    IX=[1 4 5]; IU=[]; IY=[];
    
    for k = 1:size(Vs_range,2)
        disp(['Doing descrete K: ' num2str(k) '/' num2str(size(Vs_range,2))])
        rangev = Vs_range(:,k);
        rangeh = Hs_range(:,k);
        V = Vs_trim_range(k);
        H = Hs_trim_range(k);
        X0=[V;0;0;0;H];
        [Xtrim,Utrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
        [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
        K = lqr(A,B,Q,R);
        Ks_range(:,(k-1)*5+1 : (k-1)*5+5) = K;
    end
end


%% esta es la que usa simulink dentro:
function K = state2k_discrete(state, Ks_range, Vs_range)
    % state = [V, H] instantánea
    % Ks_range = [K1, K2, K3, ...], con sus dimensiones (2,5,[])
    % Vs_range = [[V_start1;V_end1], [V_start2;V_end2],...]
    % El rango de V en el que va el controlador K3 es entre Vs_range(1,3) y
    % Vs_range(2,3).
    % K2 se usa entre [V_start2, V_end2).
    % Ejemplo: Para tener un controlador K1 entre 10 y 15 m/s, otro K2 entre
    % los 15 y los 20, otro K3 entre 20 y 25 y K4 entre 25 y 30 m/s, argumentos:
    % Ks_range = [K1, K2, K3, K4]
    % Vs_range = [[10; 15], [15;20], [20;25], [25;30]];
    
    V = state(1);
    % H = state(2);

    K = zeros(2,5);

    Ks = reshape(Ks_range,2,5,[]);
    found = 0;

    for i = 1:size(Vs_range,2)
        range = Vs_range(:,i);
        if V >= range(1) && V < range(2)
            K = Ks(:,:,i);
            found = 1;
            disp(i)
        end
    end

    if ~found
        K = zeros(2,5);
        disp('ERROR: state2k_discrete: Range not satisfied.')
    end
end
