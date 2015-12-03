function [ret] = vec_t2(b_od, v_ds, t_wait)
%VEC_T2         Calculate time spent on DS before OP
%VEC_T2(b_od, v_ds)
%   b_od        Matrix B_OD
%   v_ds        DS vector

t_ds = v_ds(:, 3)./ v_ds(:, 2) + t_wait;
ret = b_od * t_ds;

end

