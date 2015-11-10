function [ret] = vec_t4(b_dd, v_op, v_ds, mat_m, t_wait)
%VEC_T4         Calculate time spent on OP (uploading)
%               Including DS raking at OP (internal planning)
%VEC_T4(b_dd, v_op, v_ds, mat_m, t_wait)
%   b_dd        Matrix B_DD
%   v_op        OP vector
%   v_ds        DS vector (to know the size of data chunks)
%   mat_m       Plan matrix M
%   t_wait      Time to wait before connection can be established

% Similar to m_to_s (can be replaced)
%ls = mat_m * (1:size(mat_m, 2))';

%n_ds = size(b_dd, 1);
s_ds = v_ds(:, 3);
s_ds_acc = b_dd * s_ds;
r_op = v_op(:, 2);
r_ds = mat_m * r_op;
ret = s_ds_acc./ r_ds + t_wait;

end
