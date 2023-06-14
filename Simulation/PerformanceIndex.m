function PI = PerformanceIndex(outX, outU, Qin, Rin)
    X = outX.Data(:,1:5);
    U = outU.Data(:,1:2);
    t = outX.Time;
    dt = t(2:end) - t(1:end-1);
    J = 0;
    for k = 1:length(dt)
        J = J + ( X(k,:)*(Qin*X(k,:)') + U(k,:)*(Rin*U(k,:)') ) * dt(k);
    end    
    PI = 1000 ./ J;
end