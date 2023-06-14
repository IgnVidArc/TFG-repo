%% 

% GETTING THE DISCRETE CONTROLLERS FIRST:
W = 2.5; XCG = 0.33; IP = 0.1568;% Moment of inertia [kg m^2]

% Cost things -------------
Q_gen = blkdiag(1, 100, 100, 100, 10);
R_gen = blkdiag(100, 500);


% Discrete things: -------------
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
    % 2 --> tracking de both V y H.             % USAMOS ESTE.
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

% IMPORTANTE QUE SE SELECCIONE LO QUE CORRESPONDA (Discrete) EN EL MODELO ANTES DE
% EJECUTARLO
out = sim('UAVNLCL_TRACK_GS.slx');

%%
aux = 1; % 1 == no lo guardes, 2 == guarda las figuras
name = 'single';
basename = ['cap45_rampas_', name, '_' ];
if aux == 1
    figure()
    subplot(2,2,1)
    Showlog(logsout, 'v', 0)
    subplot(2,2,2)
    Showlog(logsout, 'x', 0)
    subplot(2,2,3)
    Showlog(logsout, 'h', 0)
    subplot(2,2,4)
    Showlog(logsout, 'u', 0)
elseif aux == 2
    Showlog(logsout, 'v', 1)
    mysave([basename, 'v.pdf'])
    Showlog(logsout, 'x', 1)
    mysave([basename, 'x.pdf'])
    Showlog(logsout, 'h', 1)
    mysave([basename, 'h.pdf'])
    Showlog(logsout, 'u', 1)
    mysave([basename, 'u.pdf'])
end

%%
function [] = Showlog(log, key, newfigure)
    % % signal = 1 --> Xreal <---
    % % signal = 2 --> r(t)
    % % signal = 3 --> Ureal <---
    % % signal = 4 --> Rreal <---
    % % signal = 4 --> X

    change_ylim = 0;

    if key == 'v'
        signalindex = [1, 4];
        compsindex = [1; 1];
        legkey = 'vyref';
        ykey = 'vabs';
        pos = 'northeast';
    elseif key == 'h'
        signalindex = [1, 4];
        compsindex = [5; 2];
        legkey = 'hyref';
        ykey = 'habs';
        pos = 'southeast';
    elseif key == 'x'
        signalindex = 1;
        compsindex = [2,3,4];
        legkey = '234';
        ykey = 'x';
        pos = 'northeast';
    elseif key == 'u'
        signalindex = 3;
        compsindex = 1:2;
        legkey = 'deltas';
        ykey = 'uabs';
        pos = 'southeast';
        change_ylim = 1; yl = [-0.5, 1.1];
    end
    xkey = 't';
    lnwd = 1;

    if newfigure; figure(); end
    hold on
    time = log{1}.Values.Time;
    for k = 1:length(signalindex)
        signal = signalindex(k);
        comps = compsindex(k,:);
        if key == 'u'
             plot(time, reshape(log{signal}.Values.Data(comps,1,:),length(comps),length(time)),...
             LineWidth=lnwd, LineStyle="-" )
        else
            plot(time, log{signal}.Values.Data(:,comps), LineWidth=lnwd, LineStyle="-")
        end
    end

    if change_ylim
        ylim(yl)
    end
    
    nfig = 2;
    mylegend(legkey, pos)
    myxlabel(xkey)
    myylabel(ykey)
    myplotformat

end


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
