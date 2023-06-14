% Post procesado después del tracker y esas cosas.
load('Sim_Track_AA_trim20.mat')
load('Sim_Track_AA_discrete.mat')
load('Sim_Track_AA_NN.mat')

sim_only = Sim_Track_AA_trim20;
sim_disc = Sim_Track_AA_discrete;
sim_nn = Sim_Track_AA_NN;
%%
indexr = 1;
indexu1 = 2; indexu2 = 3;
indexx1 = 4; indexx2 = 5; indexx3 = 6; indexx4 = 7; indexx5 = 8;

%%
Nsignals = 8;
% Sim_Track_AA_discrete
simul = sim_nn;

time = simul{1}.Values.Time;
%%
if time(end) > 25
    last = 12820;
    time = time(1:last);
else
    last = length(time);
end

data = zeros(length(time(1:last)), Nsignals);

for signal = 1:Nsignals
    data(:,signal) = simul{signal}.Values.Data(1:last,:);
end

%%
figure(1)
plot(time, data(1:last,indexr), LineWidth=1)
hold on
plot(time, data(1:last,indexx1), LineWidth=1)
myxlabel('t')
legend({'$r(t)$', '$V$'}, Interpreter='latex', FontSize=12)
myplotformat

%%
figure(2)
plot(time, data(1:last,indexu1), LineWidth=1)
hold on
plot(time, data(1:last,indexu2), LineWidth=1)
myxlabel('t')
mylegend('deltas')
ylabel('Variables de control')
myplotformat

%%
figure(3)
plot(time, data(1:last,indexx2), LineWidth=1)
hold on
plot(time, data(1:last,indexx3), LineWidth=1)
plot(time, data(1:last,indexx4), LineWidth=1)
myxlabel('t')
mylegend('234')
ylabel('Variables de estado [rad] [rad] [rad/s]')
myplotformat
%%
figure(4)
plot(time, data(1:last,indexx5), LineWidth=1)
myxlabel('t')
mylegend('5')
ylabel('Altura incremental, $H$ [m]')
myplotformat

