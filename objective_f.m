function [ret] = objective_f(delta)
%   Detailed explanation goes here

limit = ones(size(delta));
objective = exp(-delta / 20);
ret = bsxfun(@min, limit, objective);

end

