function errors = ResponseError(out, index)
%     % out: simout object
%     % index: vector with the components of out.Data to compute the error
%     X = out.Data(:,index)';
%     t = out.Time;
%     err = 1/t(end) * (X.^2 * t);
    X = out.Data(:,index)';
    t = out.Time;
    dt = t(2:end) - t(1:end-1);
    errors = 1/t(end) * (X(:,1:end-1).^2 * dt);
end