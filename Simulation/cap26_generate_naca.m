%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             TRABAJO DE AERODINÁMICA                             %
%                                Método de Paneles                                %
%              Fco. Javier Angulo, Ramón Albareda e Ignacio Vidal                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. ASIGNACIÓN DATOS DEL PERFIL
% Variable <option_perfil> para seleccionar el problema a analizar:
% % = 0, control:     % perfil de ejemplo para la comprobación del método.
% % = 1, ejercicio:   % perfil correspondiente al problema correspondiente al grupo.

%%% PROFILE: NACA 2410
f_max = 2;   %  (2%)
x_fmax = 40; % (40%)
t_max = 10;  % (10%)
                                                      
% Profile Magnitudes:
f_max = f_max/100; x_fmax = x_fmax/100; t_max = t_max/100;
clear X Y ZK
if (t_max<0.1) 
    name_foil = append('NACA ',num2str(f_max*100),num2str(x_fmax*10),'0',num2str(t_max*100));
else
    name_foil = append('NACA ',num2str(f_max*100),num2str(x_fmax*10),num2str(t_max*100));    
end
display(['Profile: ' name_foil])

%% 2. PROFILE GEOMETRY
    syms x
    % Ecuación de la línea media (curvatura). Expresión simbólica:
    zc = piecewise((x <  x_fmax), f_max/(x_fmax^2) * (2*x_fmax * x - x^2),...
                   (x >= x_fmax), f_max/(1-x_fmax)^2 * ( (1-2*x_fmax) + 2*x_fmax * x - x^2));
    % Ecuación del espesor. Expresión simbólica:
    ze = 5*t_max*(0.2969*sqrt(x) - 0.1260*x - 0.3516*x^2 + 0.2843*x^3 - 0.1015*x^4);
    % Pendiente de la curvatura:
    pend = atan2(diff(zc, x),1);
    % Puntos de extradós e intradós: subíndices 'U' y 'L', respectivamente.
    x_U = x - ze * sin(pend);  z_U = zc + ze * cos(pend);
    x_L = x + ze * sin(pend);  z_L = zc - ze * cos(pend);
    
    % Vectores de puntos (de extradós, iinradós y L.M) para la representación gráfica:
    M = 500;
    x_point = 0:1/M:1;
    % Corrección indeterminación en x/c = 0.4:
    if (fix(x_fmax*M)==x_fmax*M); x_point(x_fmax*M+1) = x_point(x_fmax*M+1)+1e-10; end
    x_U_point = zeros(length(x_point), 1); z_U_point = zeros(length(x_point), 1);
    x_L_point = zeros(length(x_point), 1); z_L_point = zeros(length(x_point), 1);
    z_C_point = zeros(length(x_point), 1);
    for i=1:length(x_point)
        x_U_point(i) = double(subs(x_U, x, x_point(i)));    % vector con las x de Extradós
        z_U_point(i) = double(subs(z_U, x, x_point(i)));    % vector con las z de Extradós
        x_L_point(i) = double(subs(x_L, x, x_point(i)));    % vector con las x de Intradós
        z_L_point(i) = double(subs(z_L, x, x_point(i)));    % vector con las z de Intradós
        z_C_point(i) = double(subs(zc , x, x_point(i)));    % vector con las x de la LN
    end


%% FIGURE: NACA 2410
figure(23)
p(1) = plot(x_point, z_C_point, '--', 'LineWidth', .5, 'Color', '#7A7A7A');
hold on
color_perfil = 'k';
p(2) = plot(x_U_point, z_U_point, 'LineWidth', 1., 'color', color_perfil);
p(3) = plot(x_L_point, z_L_point, 'LineWidth', 1., 'color', color_perfil);
hold off

% legend('Orientation','horizontal'); legend('Location', 'North')
% legend([p(1) p(2)],'Línea Media', 'Extradós', 'Intradós')
% legend([p(2) p(1)],'Perfil', 'Línea media')
% title(['Figura 2.1.1. Perfil ' name_foil '.']);
xlabel('$x/c$'); ylabel('$z/c$');
daspect([1 1 1])
xlim([0 1])
ylim([-0.15 0.15])
myplotformat
% mysave('cap2_naca.pdf')

%% CL DATA FROM XFOIL:
% http://airfoiltools.com/polar/details?polar=xf-naca2410-il-100000
% http://airfoiltools.com/polar/details?polar=xf-naca2410-il-200000
% http://airfoiltools.com/polar/details?polar=xf-naca2410-il-500000
data1 = readmatrix("xf-naca2410-il-100000.csv");
data2 = readmatrix("xf-naca2410-il-200000.csv");
data5 = readmatrix("xf-naca2410-il-500000.csv");

%% FIGURE: Cl vs alpha
figure(5)
plot(data1(:,1), data1(:,2), LineWidth=1)
hold on
plot(data2(:,1), data2(:,2), LineWidth=1)
plot(data5(:,1), data5(:,2), LineWidth=1)
xlabel("\'Angulo de ataque, $\alpha$ [deg]")
ylabel("Coeficeinte de sustentaci\'on, $C_l$ [--]")
legend({'$Re = 1 \cdot 10^5$', '$Re = 2 \cdot 10^5$', '$ Re = 5 \cdot 10^5 $'}, ...
    interpreter='latex', FontSize=12, Location='northwest')
xlim([-15,20])
myplotformat
% mysave('cap2_clnaca.pdf')
hold off



    
