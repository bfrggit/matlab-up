function [ret] = vec_t_comp(v_ds, v_op, mat_m, t_wait)
%VEC_T_COMP     Calculate actual complete time for OP
%VEC_T_COMP(v_ds, v_op, mat_m)
%   v_ds        DS vector
%   v_op        OP vector
%   mat_m       Plan matrix M
%   t_wait      Time to wait before connection can be established

v_mv = vec_mv(v_ds, v_op);
b_om = mat_b_om(v_op, v_mv);
b_od = mat_b_od(v_op, v_ds);
b_oo = mat_b_oo_2(v_op);
s_dd = mat_s_dd(mat_m);
b_dd = mat_b_dd_2(s_dd, v_ds);

v_t1 = vec_t1(b_om, v_mv);
v_t2 = vec_t2(b_od, v_ds, t_wait);
v_t3 = vec_t3(b_oo, v_op, v_ds, mat_m, t_wait);
t_all = v_t1 + v_t2 + v_t3;
t_op_wait = (sum(mat_m, 1) > 0)' * t_wait;
v_tc = mat_m' * v_ds(:, 3)./ v_op(:, 2) + t_op_wait;
ret = t_all + v_tc;

end

