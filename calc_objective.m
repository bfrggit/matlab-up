function [ret] = calc_objective(vec_ds, t_up)
%   Detailed explanation goes here

d_ds = vec_ds(:, 4);
x = t_up - d_ds;
ret = objective_f(x);

end

