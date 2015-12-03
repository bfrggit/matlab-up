function [ret] = vec_t3(b_oo, v_op, v_ds, mat_m, t_wait)
%VEC_T3         Calculate time spent on OP (uploading) before OP
%               Assuming all uploading tasks at one OP finish together
%               At either beginning of OP OR end of OP depending on B_OO
%VEC_T3(b_oo, v_op, v_ds, mat_m, t_wait)
%   b_oo        Matrix B_OO
%   v_op        OP vector
%   v_ds        DS vector (to know the size of data chunks)
%   mat_m       Plan matrix M
%   t_wait      Time to wait before connection can be established

s_ds = v_ds(:, 3);
s_op = (diag(s_ds) * mat_m)' * ones(size(v_ds, 1), 1);
r_op = v_op(:, 2);
%t_op_wait = (sum(mat_m, 1) > 0)' * t_wait;
%t_op = s_op./ r_op + t_op_wait;
t_op = s_op./ r_op + t_wait;
ret = b_oo * t_op;

end
