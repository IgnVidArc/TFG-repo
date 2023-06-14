% cap5x_AverageGains.m

% [Wref, XCGref, IPref] = 2.5000    0.3300    0.1568
% load("BackUp_Simulation_Dataset_5D_002.mat")

% vV   =        10        15        20        25        30      35
% vH   =         0       750      1500      2250      3000
% vW   =    1.8750    2.1875    2.5000    2.8125    3.1250
% vXCG =    0.2475    0.2888    0.3300    0.3713    0.4125
% vIP  =    0.1176    0.1372    0.1568    0.1764    0.1960

% LO VOY A HACER CON EL DATASET S (el de 6x6)
load('BackUp_Simulation_Dataset_008.mat')
vV
vH
disp(size(CONTROLLER))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ALL YOU NEED TO KNOW IS FROM HERE:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
c1 = [0 0.4470 0.7410];
c2 = [0.8500 0.3250 0.0980];
c3 = [0.9290 0.6940 0.1250];
c8 = [0.4940 0.1840 0.5560];
c5 = [0.4660 0.6740 0.1880];
c6 = [0.3010 0.7450 0.9330];
c7 = [0.6350 0.0780 0.1840];
c4 = [1 0 0];
c9 = [0 1 0];
c10 = [0 0 1];

%% intentando replicar los que salieron al azar

c1 = [0 0.4470 0.7410];
c2 = [0.8500 0.3250 0.0980];
c3 = [0.9290 0.6940 0.1250];
c7 = [243 75 246]/256; 
c9 = [0.3010 0.7450 0.9330];
c4 = [0.6350 0.0780 0.1840];
c6 = [1 0 0];
c5 = [0 1 0];
c10 = [0 0 1];
% c8 = [207, 164, 68]/256;
c8 = [0.1, 0.1, 0.1];

mycolors = [c1; c2; c3; c4; c5; c6; c7; c8; c9; c10];
% % % % colores(1,1,:) = c1;
% % % % colores(1,2,:) = c2;
% % % % colores(1,3,:) = c3;
% % % % colores(1,4,:) = c4;
% % % % colores(1,5,:) = c9;
% % % % colores(2,1,:) = c6;
% % % % colores(2,2,:) = c7;
% % % % colores(2,3,:) = c8;
% % % % colores(2,4,:) = c5;
% % % % colores(2,5,:) = c10;

% %% GENERACIÓN DE LA ESTRUCTURA QUE ME HACE FALTA
% keep = zeros(length(vV),length(vH),10);
% % figure
% for j =1:length(vH)
%     for row=1:2
%         for col=1:5
%             idx = (row-1)*5+col;
%             keep(:,j,idx) = CONTROLLER(:,j,row,col);
%         end
%     end
% end
% %% PINTADA
% colororder(mycolors)
% figure(1)
% hold on
% for j=1:length(vH)
%     plot(vV, reshape(keep(:,j,:),length(vV),10), '-s', LineWidth=1)
% end


%% GENERACIÓN DE LA ESTRUCTURA QUE ME HACE FALTA
keep = zeros(length(vV),length(vH),10);
% figure
for j =1:length(vH)
    for row=1:2
        for col=1:5
            idx = (col-1)*2+row;
            keep(:,j,idx) = CONTROLLER(:,j,row,col);
        end
    end
end
%%
%% PINTADA

caso = 'mat';
caso = 'av';

if contains(caso, 'mat')
   yname = 'Ganancias del controlador';
   wantmat = 1; wantav = 0;
elseif contains(caso, 'av')
    yname = "Variaci\'on de las ganancias [\%]";
    wantmat = 0; wantav = 1;
end

close all
figure(1)
colororder(mycolors)
hold on
for j=1:length(vH)
    mat = reshape(keep(:,j,:),length(vV),10); % [6,10], 6 velocidades, 10 ganancias.
    av = (mat - 0*mean(mat,1)) ./ mean(mat,1) * 100;
    result = wantmat * mat + wantav * av;
    plot(vV, result, '-s', LineWidth=1)
end
% ylim([-4, 5]);
legend({'$k_{11}$', '$k_{21}$', ...
        '$k_{12}$', '$k_{22}$', ...
        '$k_{13}$', '$k_{23}$', ...
        '$k_{14}$', '$k_{24}$', ...
        '$k_{15}$', '$k_{25}$',}, Interpreter="latex", NumColumns=5, Location='north', ...
        fontsize=12, AutoUpdate='off')

% legend({'$k_{11}$', '$k_{12}$', '$k_{13}$', '$k_{14}$', '$k_{15}$', ...
%         '$k_{21}$', '$k_{22}$', '$k_{23}$', '$k_{24}$', '$k_{25}$' ...
%         }, Interpreter="latex", NumColumns=5, Location='northwest', ...
%         fontsize=12)

nfig = 1;
myxlabel('v')
ylabel(yname, Interpreter="latex", FontSize=15)
xlim([vV(1)-1, vV(end)+1])
myplotformat


%%
% mysave('cap52_lafiguradedios.pdf')

