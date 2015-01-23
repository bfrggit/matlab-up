function [ret] = mat_b_od(v_op, v_ds)
%MAT_B_OD       Calculate matrix B_OD (DS no later than OP)
%MAT_B_OD(v_op, v_ds)
%   v_op        OP vector
%   v_ds        DS vector

ret = mat_b_xy(v_op(:, 1), v_ds(:, 1));

end

