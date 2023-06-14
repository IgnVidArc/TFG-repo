function PI = PerformanceIndexMod(outX, outU, outUreal, Qin, Rin)
    % Calculates the PI = 1/J.
    % New modificaton:
    % The elevator cost is calculated with the incremental control variable.
    % The thrust cost is calculated with the absolute control variable.

    % Data:
    X = outX.Data(:,1:5);
    U = outU.Data(:,1:2);
    Ureal = outUreal.Data(:,1:2);
    % Time:
    t = outX.Time;
    dt = t(2:end) - t(1:end-1);
    % Modified (absolute thrust, relative elevator)
    Umod = [Ureal(:,1), U(:,2)];
    J = 0;
    for k = 1:length(dt)
        J = J + (X(k,:)*(Qin*X(k,:)') + Umod(k,:)*(Rin*Umod(k,:)'))*dt(k);
    end
    PI = 1 ./ J;
end