function [ret] = mat_b_dd_2(s_dd, v_ds)
%MAT_B_DD_2     Calculate matrix S_DD (DS at same OP no later than DS)
%MAT_B_DD_2(s_dd, v_ds)
%   s_dd        Matrix S_DD
%   v_ds        DS vector

n_ds = size(s_dd, 1);
%seq = 1:n_ds;
%triangle = (repmat(seq, n_ds, 1) < repmat(seq', 1, n_ds)) + eye(n_ds);
h_deadline = v_ds(:, 4);
h_priority = v_ds(:, 5);
h_num = (1:1:n_ds)';

td_ds = repmat(h_deadline, 1, n_ds) > repmat(h_deadline', n_ds, 1);
tddds = repmat(h_deadline, 1, n_ds) == repmat(h_deadline', n_ds, 1);
tp_ds = repmat(h_priority, 1, n_ds) < repmat(h_priority', n_ds, 1);
tppds = repmat(h_priority, 1, n_ds) == repmat(h_priority', n_ds, 1);
tn_ds = repmat(h_num, 1, n_ds) < repmat(h_num', n_ds, 1);
ret = ((tp_ds | tppds & (td_ds | tddds & tn_ds)) & s_dd) + eye(n_ds);

end

