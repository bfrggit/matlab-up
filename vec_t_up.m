function [ret] = vec_t_up(v_ds, v_op, mat_m)
%VEC_T_UP       Calculate actual upload time for DS
%VEC_T_UP(v_ds, v_op, mat_m)
%   v_ds        DS vector
%   v_op        OP vector
%   mat_m       Plan matrix M

v_mv = vec_mv(v_ds, v_op);
b_om = mat_b_om(v_op, v_mv);
b_od = mat_b_od(v_op, v_ds);
b_oo = mat_b_oo_2(v_op);
s_dd = mat_s_dd(mat_m);
%b_dd = mat_b_dd(s_dd);
b_dd = mat_b_dd_2(s_dd, v_ds);

v_t1 = vec_t1(b_om, v_mv);
v_t2 = vec_t2(b_od, v_ds);
v_t3 = vec_t3(b_oo, v_op, v_ds, mat_m);
t_all = v_t1 + v_t2 + v_t3;
v_t4 = vec_t4(b_dd, v_op, v_ds, mat_m);
ret = mat_m * t_all + v_t4;

end

