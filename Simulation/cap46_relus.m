x1 = -10:1:9
x2 = [-9.50000000000000	-8.50000000000000	-7.50000000000000	-6.50000000000000	-5.50000000000000	-4.50000000000000	-3.50000000000000	-2.50000000000000	-1.50000000000000	-0.500000000000000	0 0.500000000000000	1.50000000000000	2.50000000000000	3.50000000000000	4.50000000000000	5.50000000000000	6.50000000000000	7.50000000000000	8.50000000000000	9.50000000000000];
% x2 = 
a = 0.1;
relu = max(0,x1);
leaky = max(a*x2,x2);
figure(1)
plot(x1,relu,'-s')
hold on
plot(x2,leaky, '-o')
%%
close all
x = -1:0.1:1;
a = [0.05, 0.1, 0.2]';
relu = max(0,x);
leaky = max(a.*x,x);
figure(2)
plot(x,relu,'-', LineWidth=1)
hold on
plot(x,leaky, '-', LineWidth=1)
nfig=2;
myxlabel('x')
myylabel('relus')
mylegend('relus', 'northwest')
myplotformat
% mysave('cap46_relus.pdf')
