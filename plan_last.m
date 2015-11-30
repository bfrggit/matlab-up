function [m, ls] = plan_last(v_ds, v_op)
%PLAN_LAST      Generate a plan using only last OP, for comparison
%PLAN_LAST(v_ds, v_op)
%   v_ds        DS vector
%   v_op        OP vector

[m_asap, l_asap] = plan_asap(v_ds, v_op);
n_op = size(v_op, 1) - 1;
n_ds = size(v_ds, 1);
l_last = repmat(n_op, n_ds, 1);
ls = max([l_asap l_last], [], 2);
m = ls_to_m(ls, n_op + 1);

end

