%% COMPARING THE RESPONSE WITH A PROPPER LQR WITH THE PREDICTED STATE2K CONTROLLER


% chunk == 1 --> parte de comparar PI a varias velocidades.
% chunk == 2 --> parte de generar un plot comparando dos respuestas.

chunk = 2;

if chunk==1

    W = 2.5;
    XCG = 0.33;
    Q0 = blkdiag(1, 100, 100, 0, 0);
    R0 = blkdiag(100, 100);
    
    variousV = [10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30];
    H = 1000;
    PIS_LQR = zeros(length(variousV), length(variousH));
    PIS_NN  = zeros(length(variousV), length(variousH));
    
    x0 = [-1, .0, .5, 0.1, -1.5];
    StopTime = 5;
    
    for k=1:length(variousV)
        disp(['(V,H) = (' num2str(k) '/' num2str(length(variousV)) ', ' num2str(h) '/' num2str(length(variousH)) ')'])
    
        state = [variousV(k), H];
        
        V = state(1);  
        H = state(2);
        X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
        IX=[1 4 5];     IU=[];         IY=[];
        [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
        [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);
        
        
        % With LQR controller
        Q = blkdiag(1, 100, 100, 100, 10);
        R = blkdiag(100, 500);
        K = lqr(A,B,Q,R);
        sim("UAVLinCL_DEF_v0");
        logsorig = logsout;    
        PIS_LQR(k) = PerformanceIndex(outX, outU, Q0, R0);
        
        % With Predicted controller
        K = state2k(state);
        sim("UAVLinCL_DEF_v0");
        logsnn = logsout;
        PIS_NN(k) = PerformanceIndex(outX, outU, Q0, R0);
    
    end

    close all
    figure
    hold on
    
    markers = {'-s', '-s'};
    mksz = 7;
    lnwd = 1.5;
    plot(variousV, PIS_LQR, markers{1}, MarkerSize=mksz, LineWidth=lnwd)
    plot(variousV,  PIS_NN, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
    % plot(variousV,  abs(PIS_NN-PIS_LQR), markers{2}, MarkerSize=mksz, LineWidth=lnwd)
    
    nfig = 1;
    if nfig==1
        legfontsize = 12;
        axisfontsize = 14;
    elseif nfig==2
        legfontsize = 15;
        axisfontsize = 17;
    end
    legend({'LQR', 'NN'}, Interpreter='latex', FontSize=legfontsize)
    myxlabel('v')
    ylabel({"\'Indice de Desempe\~no"}, Interpreter='latex', FontSize=axisfontsize)
    myplotformat
    
    % mysave('cap4_PIvsPI.pdf')
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif chunk == 2
    W = 2.5;
    XCG = 0.33;
    Q0 = blkdiag(1, 100, 100, 0, 0);
    R0 = blkdiag(100, 100);

    state = [28, 1000];

    x0 = [-1, .0, .5, 0.1, -1.5];
    StopTime = 5;

    V = state(1);  
    H = state(2);
    X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
    IX=[1 4 5];     IU=[];         IY=[];
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);


    % With LQR controller
    Q = blkdiag(1, 100, 100, 100, 10);
    R = blkdiag(100, 500);
    K = lqr(A,B,Q,R);
    sim("UAVLinCL_DEF_v0");
    logsorig = logsout;
    outX_LQR = outX;
    outU_LQR = outU;
    PI_LQR = PerformanceIndex(outX, outU, 1*Q0, 1*R0);

    % With Predicted controller
    K = state2k(state);
    sim("UAVLinCL_DEF_v0");
    logsnn = logsout;
    outX_NN = outX;
    outU_NN = outU;
    PI_NN = PerformanceIndex(outX, outU, Q0, R0);


    close all
    Showlog([logsorig, logsnn], 'x')
    ylim([-3,3])
    % mysave('cap4_comp_nn_high_x.pdf')

    Showlog([logsorig, logsnn], 'ureal')
    ylim([-0.5,1])
    % mysave('cap4_comp_nn_high_u.pdf')


end


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



