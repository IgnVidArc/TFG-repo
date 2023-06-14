% TO RUN BEFORE RUNNING UAVNLCL_TRACK_GS.slx:
load('BackUp_Simulation_Dataset_006.mat')
%%
load('TableData_006.mat')
%%              
        % Nv = size(CONTROLLER,1);
        % Nh = size(CONTROLLER,2);
        % TABLEDATA = zeros(2,5,Nv,Nh);
        % for i = 1:Nv
        %     for j = 1:Nh
        %         TABLEDATA(:,:,i,j) = CONTROLLER(i,j,:,:);
        %     end
        % end
        % save('TableData_006.mat', "TABLEDATA")
%%
% Cost things -------------
Q_gen = blkdiag(1, 1000, 1000, 100, 10);
R_gen = blkdiag(100, 500);

% RAMP THNIGS -------------
Vstart = 10;
path_angle = 7;
rampH = Vstart*tan(path_angle*pi/180);
rawslopes = [1, rampH];

%% THE OTHER PART OF THE MODEL:
% Additional parameters:
W = 2.5; XCG = 0.33;
% Trim conditions:
V = Vstart;
H = 1000;
X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];
% Trim and linear model:
[Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
[A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);

% LQR Controller:
[K,S,P] = lqr(A,B,Q_gen,R_gen);

x0 = zeros(5,1);

% Este script con lo que funciona es con el UAVNLCL_TRACK_GS.slx.

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

sim('UAVNLCL_TRACK_GS.slx')
%%
figure()
subplot(2,2,1)
Showlog(logsout, 'v', 0)
subplot(2,2,2)
Showlog(logsout, 'x', 0)
subplot(2,2,3)
Showlog(logsout, 'h', 0)
subplot(2,2,4)
Showlog(logsout, 'u', 0)

function [] = Showlog(log, key, newfigure)
    % % signal = 1 --> Xreal ---
    % % signal = 2 --> r(t)
    % % signal = 3 --> Ureal ---
    % % signal = 4 --> Rreal ---
    % % signal = 4 --> X

    if key == 'v'
        signalindex = [1, 4];
        compsindex = [1; 1];
        legkey = 'vyref';
        ykey = 'vabs';
    elseif key == 'h'
        signalindex = [1, 4];
        compsindex = [5; 2];
        legkey = 'hyref';
        ykey = 'habs';
    elseif key == 'x'
        signalindex = 1;
        compsindex = [2,3,4];
        legkey = '234';
        ykey = 'x';
    elseif key == 'u'
        signalindex = 3;
        compsindex = 1:2;
        legkey = 'deltas';
        ykey = 'uabs';        
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
             plot(time, reshape(log{signal}.Values.Data(comps,1,:),length(comps),length(time)), ...
             LineWidth=lnwd, LineStyle="-")
        else
            plot(time, log{signal}.Values.Data(:,comps), LineWidth=lnwd, LineStyle="-")
        end
    end
    
    nfig = 2;
    mylegend(legkey)
    myxlabel(xkey)
    myylabel(ykey)
    myplotformat

end


