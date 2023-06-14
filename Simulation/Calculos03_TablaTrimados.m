%% 3. TABLA DE TRIMADOS, BUCLE

% Cosas genéricas conservadas:
W=2.5;
XCG=0.33;      
H=100;      

% Vector de velocidades:
vV = 1:40;
vH = 100:3000
vthrust = zeros(length(vV),1);
%%
for i = 1:length(vV)
    for k = 1:length(vH)
        V = vV(k);
        X0=[V;0;0;0;H]; U0=[0.6;-0.1]; Y0=[];
        IX=[1 4 5];     IU=[];         IY=[];
        [Xtrim,Utrim,Ytrim,DXtrim] = trim('UAVTrimh',X0,U0,Y0,IX,IU,IY);
        vthrust(k) = Utrim(1);
    end
end

plot(vV, vthrust)


%%
% CLopt = sqrt()
% VB = sqrt(2*W/(density(H)*S*CLopt))
% Trim:
% Bueno, que sí, que muy probablemente esa sea la velocidad base.






