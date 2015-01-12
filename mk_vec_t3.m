function [ret] = mk_vec_t3(b_oo, vec_op, vec_ds, mat_m)
%   Detailed explanation goes here

s_ds = vec_ds(:, 3);
s_op = (diag(s_ds) * mat_m)' * ones(size(vec_ds, 1), 1);
r_op = vec_op(:, 2);
t_op = s_op./ r_op;
ret = b_oo * t_op;

end

