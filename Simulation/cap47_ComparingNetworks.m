%% COMPARING THE RESPONSE WITH A PROPPER LQR WITH THE PREDICTED STATE2K CONTROLLER


W = 2.5;
XCG = 0.33;
Q0 = blkdiag(1, 100, 100, 0, 0);
R0 = blkdiag(100, 100);

variousV = [10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30];
H = 1000;

PIS_LQR = zeros(length(variousV), length(H));


x0 = [-1, .0, .5, 0.1, -1.5];
StopTime = 5;
% LOOP FOR THE ORIGINAL LQR CONTROLLER
for k=1:length(variousV)
    disp(['(V) = (' num2str(k) '/' num2str(length(variousV)) ')'])

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

end


%% CREATING VARIABLE WITH SPECIF MODEL PI FROM state2k


variousV = [10, 12.5, 15, 17.5, 20, 22.5, 25, 27.5, 30];
H = 1000;

PIS_NN = zeros(length(variousV), length(H));

disp('----------')
for k=1:length(variousV)
    disp(['(V) = (' num2str(k) '/' num2str(length(variousV)) ')'])

    state = [variousV(k), H];

    V = state(1);
    H = state(2);
    X0=[V;0;0;0;H]; U0=[0.5;-0.1]; Y0=[];
    IX=[1 4 5];     IU=[];         IY=[];
    [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
    [A,B,C,D]=linmod('UAVTrimh',Xtrim,Utrim);


    % With Predicted controller
    K = state2k(state);
    sim("UAVLinCL_DEF_v0");
    logsnn = logsout;
    PIS_NN(k) = PerformanceIndex(outX, outU, Q0, R0);

end

% PIS_NN_006_10 = PIS_NN;


% err_005_20_40_40_20_drop04 = 1/length(variousV) * sum(abs(PIS_LQR - PIS_NN_005_20_40_40_20_drop04));
errB3 = 1/length(variousV) * sum(abs(PIS_LQR - PIS_NN))

%%
[err_005_20_40_40_20_drop01, err_005_20_40_40_20_drop02, err_005_20_40_40_20_drop03, err_005_20_40_40_20_drop04]'


%% DROP-P

close all
figure
hold on

markers = {'-s', '-s', '--o'};
mksz = 7;
lnwd = 1.5;
plot(variousV, PIS_LQR, markers{1}, MarkerSize=mksz, LineWidth=lnwd)

plot(variousV,  PIS_NN_005_20_40_40_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_40_40_20_drop02, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_40_40_20_drop03, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_40_40_20_drop04, markers{2}, MarkerSize=mksz, LineWidth=lnwd)

nfig = 1;
if nfig==1
    legfontsize = 12;
    axisfontsize = 14;
elseif nfig==2
    legfontsize = 15;
    axisfontsize = 17;
end
identifiers = {'LQR',...
               'NN: {drop}\_p = 0.1', ...
               'NN: {drop}\_p = 0.2', ...
               'NN: {drop}\_p = 0.3', ...
               'NN: {drop}\_p = 0.4', ...
               };
legend(identifiers, Interpreter='latex', FontSize=legfontsize, Location='southeast')
myxlabel('v')
ylabel({"\'Indice de Desempe\~no"}, Interpreter='latex', FontSize=axisfontsize)
myplotformat

%% all of initial them

close all
figure
hold on

markers = {'-s', '-s', '--o'};
mksz = 7;
lnwd = 1.5;
plot(variousV, PIS_LQR, markers{1}, MarkerSize=mksz, LineWidth=lnwd)

plot(variousV,  PIS_NN_005_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_40_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_40_40_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)

plot(variousV,  PIS_NN_006_20, markers{3}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_006_20_20, markers{3}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_006_20_40_20, markers{3}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_006_20_40_40_20, markers{3}, MarkerSize=mksz, LineWidth=lnwd, Marker='o')


nfig = 1;
if nfig==1
    legfontsize = 12;
    axisfontsize = 14;
elseif nfig==2
    legfontsize = 15;
    axisfontsize = 17;
end
identifiers = {'LQR',...
               'NN: A/20', ...
               'NN: A/20/20', ...
               'NN: A/20/40/20', ...
               'NN: A/20/40/40/20', ...
               'NN: B/20', ...
               'NN: B/20/20', ...
               'NN: B/20/40/20', ...
               'NN: B/20/40/40/20', ...
               };
legend(identifiers, Interpreter='latex', FontSize=legfontsize, Location='best')
myxlabel('v')
ylabel({"\'Indice de Desempe\~no"}, Interpreter='latex', FontSize=axisfontsize)
myplotformat


%% SOLO CON DATASET 5
close all
figure
hold on

markers = {'-s', '-s'};
mksz = 7;
lnwd = 1.5;
plot(variousV, PIS_LQR, markers{1}, MarkerSize=mksz, LineWidth=lnwd)

plot(variousV,  PIS_NN_005_10, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_40_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_005_20_40_40_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)


nfig = 1;
if nfig==1
    legfontsize = 12;
    axisfontsize = 14;
elseif nfig==2
    legfontsize = 15;
    axisfontsize = 17;
end
identifiers = {'LQR',...
               'NN: (10)', ...
               'NN: (20)', ...
               'NN: (20/20)', ...
               'NN: (20/40/20)', ...
               'NN: (20/40/40/20)', ...
               };
legend(identifiers, Interpreter='latex', FontSize=legfontsize, Location='northwest')
myxlabel('v')
ylabel({"\'Indice de Desempe\~no"}, Interpreter='latex', FontSize=axisfontsize)
myplotformat

% mysave('cap4_PI_dsA.pdf')


%% SOLO CON DATASET 6
close all
figure
hold on

markers = {'-s', '-s'};
mksz = 7;
lnwd = 1.5;
plot(variousV, PIS_LQR, markers{1}, MarkerSize=mksz, LineWidth=lnwd)

plot(variousV,  PIS_NN_006_10, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_006_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_006_20_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_006_20_40_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)
plot(variousV,  PIS_NN_006_20_40_40_20, markers{2}, MarkerSize=mksz, LineWidth=lnwd)


nfig = 1;
if nfig==1
    legfontsize = 12;
    axisfontsize = 14;
elseif nfig==2
    legfontsize = 15;
    axisfontsize = 17;
end
identifiers = {'LQR',...
               'NN: (10)', ...
               'NN: (20)', ...
               'NN: (20/20)', ...
               'NN: (20/40/20)', ...
               'NN: (20/40/40/20)', ...
               };
legend(identifiers, Interpreter='latex', FontSize=legfontsize, Location='northwest')
myxlabel('v')
ylabel({"\'Indice de Desempe\~no"}, Interpreter='latex', FontSize=axisfontsize)
myplotformat

% mysave('cap4_PI_dsB.pdf')

%%
PIS_important_note = "Esto son los primeros que hago. ";
save('PIS_NN_cap45_v0', "PIS_important_note", ...
    "PIS_NN_006_20_40_40_20", "PIS_NN_006_20", "PIS_NN_006_20_20", "PIS_NN_006_20_40_20", ...
    "PIS_NN_005_20", "PIS_NN_005_20_20", "PIS_NN_005_20_40_20", "PIS_NN_005_20_40_40_20", '-mat')



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



