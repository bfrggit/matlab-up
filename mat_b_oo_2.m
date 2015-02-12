function [ret] = mat_b_oo_2(v_op)
%MAT_B_OO_2     Calculate matrix B_OO (OP before OP)
%MAT_B_OO_2(v_op)
%   v_op        OP vector

ret = mat_b_xy(v_op(:, 1), v_op(:, 1)); %+ eye(size(vec_op, 1));

end

