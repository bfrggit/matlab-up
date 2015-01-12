function [ret] = mk_vec_t2(b_od, vec_ds)
%   Detailed explanation goes here

t_ds = vec_ds(:, 3)./ vec_ds(:, 2);
ret = b_od * t_ds;

end

