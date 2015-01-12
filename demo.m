mat_m = plan_asap(vec_ds, vec_op);

vec_mv = mk_vec_mv(vec_ds, vec_op);
b_om = mk_b_om(vec_op, vec_mv);
b_od = mk_b_od(vec_op, vec_ds);
b_oo = mk_b_oo(vec_op);
vec_t1 = mk_vec_t1(b_om, vec_mv);
vec_t2 = mk_vec_t2(b_od, vec_ds);
vec_t3 = mk_vec_t3(b_oo, vec_op, vec_ds, mat_m);
t_all = vec_t1 + vec_t2 + vec_t3;
t_up = mat_m * t_all;
reward_each = calc_objective(vec_ds, t_up);
reward = sum(reward_each.* vec_ds(:, 5)) / sum(vec_ds(:, 5));
