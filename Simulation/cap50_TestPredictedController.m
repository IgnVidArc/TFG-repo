
% [Wref, XCGref, IPref] = 2.5000    0.3300    0.1568
load("BackUp_Simulation_Dataset_5D_002.mat")

% vV   =        10        15        20        25        30      35
% vH   =         0      1200      2400      3600      4800    6000
% vW   =    1.8750    2.1875    2.5000    2.8125    3.1250
% vXCG =    0.2475    0.2888    0.3300    0.3713    0.4125
% vIP  =    0.1176    0.1372    0.1568    0.1764    0.1960
%%


states = [21.0, 2118, 3.40, .39, .18];
Nstates = size(states,1);
% K = state2k(state(1,:);

% Trim conditions
% X0=[state(1);0;0;0;state(2)]; U0=[1;0.5]; Y0=[];
% IX=[1 4 5];     IU=[];         IY=[];
% % Trim:
% [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh5D',X0,U0,Y0,IX,IU,IY);
% % Linear Model:
% [A,B,C,D]=linmod('UAVTrimh5D',Xtrim,Utrim);

StopTime = 5;
x0 = [-1, .0, .5, 0.1, -1.5];
Q = blkdiag(1, 100, 100, 100, 10);
R = blkdiag(100, 500);

U0=[1;0.5]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];

for k=1:Nstates
    state = states(k,:);
    X0=[state(1);0;0;0;state(2)];
    W   = state(3);
    XCG = state(4);
    IP  = state(5);
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh5D',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh5D',Xtrim,Utrim);   
    K = lqr(A,B,Q,R);
    sim('UAVLinCL_DEF_v0')
    log_lqr(k) = logsout;
    K = state2k(state);
    sim('UAVLinCL_DEF_v0')
    log_nn(k) = logsout;
end

% Showlog([log_lqr(1)], 'x')
Showlog([log_lqr(1), log_nn(1)], 'x')
Showlog([log_lqr(1), log_nn(1)], 'ureal')

%%

function [] = Showlog(logs, signal)
    % % signal = 1 --> u
    % % signal = 2 --> x
    % % % % % signal = 3 --> ureal

    if signal == 'x'
        signalindex = 2;
        legkey = 'allxboth';
        ykey = 'x';
    elseif contains(signal,'ureal')
        signalindex = 3;
        legkey = 'deltasboth';
        ykey = 'uabs';
    elseif signal == 'u'
        signalindex = 1;
        legkey = 'deltas';
        ykey = 'u';
    
    end
    xkey = 't';

    styles = {'--', '-'};
    
    figure()
    for idx=1:length(logs)
        log = logs(idx);
        style = styles{idx};
        
        time = log{1}.Values.Time;
        plot(time, log{signalindex}.Values.Data(:,:), LineWidth=1, LineStyle=style)
        hold on

    end
    nfig = 2;
    mylegend(legkey)
    myxlabel(xkey)
    myylabel(ykey)
    myplotformat

end

