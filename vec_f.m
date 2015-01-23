function [ret] = vec_f(v_ds, t_up)
%VEC_F          Calculate objective function value for each DS
%VEC_F(v_ds, t_up)
%   v_ds        DS vector
%   t_up        Actual upload time vector

d_ds = v_ds(:, 4);
x = t_up - d_ds;
ret = objective_f(x);

end

