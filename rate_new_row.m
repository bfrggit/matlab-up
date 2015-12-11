function [ret] = rate_new_row(v_ds, t_up)
%RATE_NEW_ROW   Calculate success rates (and return in one row)
%RATE_NEW_ROW(v_ds, t_up)
%   v_ds        DS vector (to get priority values)
%   v_f         Reward vector got from objective function call

rate_new_ret = rate_new(v_ds, t_up);
v_all_p = rate_new_ret(:, 1);
v_suc_p = rate_new_ret(:, 2);

ret = [v_all_p', v_suc_p'];

end

