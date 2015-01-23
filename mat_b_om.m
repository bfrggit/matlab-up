function [ret] = mat_b_om(v_op, v_mv)
%MAT_B_OM       Calculate matrix B_OM (MV no later than OP)
%MAT_B_OM(v_op, v_mv)
%   v_op        OP vector
%   v_ds        DS vector

ret = mat_b_xy(v_op(:, 1), v_mv(:, 1));

end

