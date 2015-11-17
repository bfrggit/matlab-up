function [ret] = objective_f(delta)
%OBJECTIVE_F    Objective function definition
%OBJECTIVE_F(delta)
%   delta       Time elapsed from deadline to actual delivery time

limit = ones(size(delta));
objective = exp(-delta / 30 * log(2));
ret = bsxfun(@min, limit, objective);

end

