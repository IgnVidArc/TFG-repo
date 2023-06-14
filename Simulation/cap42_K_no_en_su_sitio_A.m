% para ver un controlador de un sitio en otro sitio.

%% PRIMER CONTROLADOR
W = 2.5; XCG = 0.33;
states = [15, 3000; ... A
          27, 500];... B
Nstates = size(states,1);
Ks = zeros(2,5,Nstates);

U0=[0.5;-0.1]; Y0=[];
IX=[1 4 5];     IU=[];         IY=[];

Q = blkdiag(1, 100, 100, 100, 10);
R = blkdiag(100, 500);

StopTime = 5;
x0 = [-1, .0, .5, 0.1, -1.5];
%%

for k=1:Nstates
    X0=[states(k,1);0;0;0;states(k,2)];
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);   
    K = lqr(A,B,Q,R);
    Ks(:,:,k) = K;
    sim('UAVLinCL_DEF_v0')
    logss_own(k) = logsout;
end
%%
close all
% using the controller of this state and the other
which_K = 2;

if which_K == 1
    controllername = 'A';
elseif which_K == 2
    controllername = 'B';
end
PIJ  = zeros(Nstates,1);
PIx = zeros(Nstates,1);
PIu = zeros(Nstates,1);

aux1 = [1,2];
aux2 = [3,4];

figname = ['cap4_controller' controllername '_en'];
      
x0 = [-1, .0, .5, 0.1, -1.5];

for k=1:Nstates
    if k==1; point = 'A';
    elseif k==2; point = 'B';
    end
    % Condition:
    % x0 = [-1, .0, .5, 0.1, -1.5]*states(k,1)/states(2,1);

    X0=[states(k,1);0;0;0;states(k,2)];
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);   
    K = Ks(:,:,which_K);
    sim('UAVLinCL_DEF_v0')
    % subplot(2,2,aux1(k))
    Showlog(logsout, 'x')
    ylim([-3,3])
    % % % mysave([figname point '_x.pdf'])
    % subplot(2,2,aux2(k))
    Showlog(logsout, 'ureal')
    ylim([-0.5,1])
    % % % mysave([figname point '_u.pdf'])
    logss_guiris(k) = logsout;
    PIJ(k) = PerformanceIndex(outX, outU, Q, R);
    PIx(k) = PerformanceIndex(outX, outU, Q, 0*R);
    PIu(k) = PerformanceIndex(outX, outU, 0*Q, R);
end

PIJ
PIx
PIu

%%

function [] = Showlog(log, signal)
    % % signal = 1 --> u
    % % signal = 2 --> x
    % % % % % signal = 3 --> ureal

    if signal == 'x'
        signalindex = 2;
        legkey = 'allx';
        ykey = 'x';
    elseif contains(signal,'ureal')
        signalindex = 3;
        legkey = 'deltas';
        ykey = 'uabs';
    elseif signal == 'u'
        signalindex = 1;
        legkey = 'deltas';
        ykey = 'u';
    
    end
    xkey = 't';

    time = log{1}.Values.Time;
    figure()
    plot(time, log{signalindex}.Values.Data(:,:), LineWidth=1)
    nfig = 2;
    mylegend(legkey)
    myxlabel(xkey)
    myylabel(ykey)
    myplotformat

end

