function [ret] = mat_b_oo(vec_op)
%MAT_B_OO       Calculate matrix B_OO (OP no later than OP)
%MAT_B_OO(vec_op)
%   v_op        OP vector

ret = mat_b_xy(vec_op(:, 1), vec_op(:, 1)) + eye(size(vec_op, 1));

end

