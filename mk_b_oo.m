function [ret] = mk_b_oo(vec_op)
%   Detailed explanation goes here

ret = mk_mat_b_xy(vec_op(:, 1), vec_op(:, 1)) + eye(size(vec_op, 1));

end

