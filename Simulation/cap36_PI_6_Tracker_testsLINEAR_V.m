%% 
close all
% GETTING THE DISCRETE CONTROLLERS FIRST:
W = 2.5;
XCG = 0.33;

% Cost things -------------
Q_gen = blkdiag(1, 100, 100, 100, 10);
R_gen = blkdiag(100, 500);
% A(sub2ind(size(A),1:size(A,1),1:size(A,2)))


% Discrete things: -------------
Vs_range = [[10; 15], [15;20], [20;25], [25;30]];
Vs_range = [8; 45];
Vs_trim_range = 19;     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Hs_range = zeros(size(Vs_range));
Hs_range(:,:) = 1000;
Hs_trim_range = Hs_range(1,:);
Ks_range = get_Ks_range(Vs_range, Vs_trim_range, Hs_range, Hs_trim_range, Q_gen, R_gen);

% STEP THNIGS:
step_amplitude = 2;
step_period = 10;

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
% Q = blkdiag(1, 1000, 1000, 100, 1);
% R = blkdiag(100, 100);
[K,S,P] = lqr(A,B,Q_gen,R_gen);

x0 = zeros(5,1);

% Este script con lo que funciona es con el UAVNLCL_TRACK_NN.slx.

% TRAKING:
q_xi_V = 100;
% q_xi_H = 10;

caso = 1;
    % 1 --> solo tracking de la velociadad
    % 2 --> tracking de both V y H.
    % 3 --> era solo H, pero abrir el otro .m con los pots para el track de H.
h0t = 0;
if caso == 1
    slopes = rawslopes(1);
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
    % h0t = 1;
    slopes = 0;
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
StopTime = 50;
out = sim('UAVLinCL_tracking_Vsquare.slx');
%% PARA PLOT VELOCIDAD Y r(t) CUANDO ES ELLA LA QUE SEGUIMOS
figure(1)
if caso==1
    plot(out.tout, Xtrim(1) + out.outX(:,1), linewidth=1)
    hold on
    plot(out.tout, Xtrim(1) +  out.logsout{1}.Values.Data(:,1), linewidth=1)
    % xlim([0,35])
    nfig = 2;
    myxlabel('t')
    myylabel('vabs')
    % ylabel('Velocidad absoluta [m/s]', Interpreter='latex')
    legend({'V', '$r(t)$'}, Interpreter="latex", FontSize=12)
    myplotformat
    % mysave('cap3_trackV_a.pdf')
elseif caso==3
    figure(1)
    plot(out.tout, out.outX(:,1), linewidth=1)
    % hold on
    % plot(out.tout, Xtrim(1) +  out.logsout{1}.Values.Data(:,1), linewidth=1)
    % xlim([0,35])
    nfig = 2;
    myxlabel('t')
    myylabel('v')
    % ylabel('Velocidad absoluta [m/s]', Interpreter='latex')
    legend({'V'}, Interpreter="latex", FontSize=12)
    myplotformat
end

%%
figure(2)
plot(out.tout, out.outX(:,2:4), linewidth=1)
% xlim([0,35])
nfig = 2;
myxlabel('t')
myylabel('allx')
% ylabel('Velocidad absoluta [m/s]', Interpreter='latex')
% legend({'V', '$r(t)$'}, Interpreter="latex", FontSize=12)
mylegend('234')
myplotformat
% mysave('cap3_trackV_b.pdf')

%%

if caso==1
    figure(3)
    plot(out.tout, Xtrim(5) + out.outX(:,5), linewidth=1)
    % xlim([0,35])
    nfig = 2;
    myxlabel('t')
    myylabel('habs')
    mylegend('h')
    myplotformat
    % mysave('cap3_trackV_c.pdf')
elseif caso==3
    % figure(3)
    % plot(out.tout, out.outX(:,5), linewidth=1)
    % hold on
    % plot(out.tout, out.logsout{1}.Values.Data(:,1), linewidth=1)
    % % xlim([0,35])
    % nfig = 2;
    % myxlabel('t')
    % myylabel('h')
    % mylegend('h')
    % myplotformat
    % mysave('cap3_trackV_c.pdf')
end


%%
figure(4)
plot(out.tout, [Utrim(1) + out.outU(:,1), Utrim(2) + out.outU(:,2)], linewidth=1)
% xlim([0,35])
nfig = 2;
myxlabel('t')
myylabel('uabs')
mylegend('deltas')
myplotformat
% mysave('cap3_trackV_d.pdf')

%%
% state = [10, H]; 
% Ktest = state2k_discrete(state, Ks_range, Vs_range);
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
