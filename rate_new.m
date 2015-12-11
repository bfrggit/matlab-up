function [ret] = rate_new(v_ds, t_up)
%RATE_NEW       Calculate success rates
%RATE_NEW(v_ds, t_up)
%   v_ds        DS vector (to get priority values)
%   v_f         Reward vector got from objective function call

%global P_DIST;
%v_pr = P_DIST(:, 1);

d_ds = v_ds(:, 4);
p_ds = v_ds(:, 5);

% Matrix to identify priority of each DS
mat_pd = zeros(size(v_ds, 1), 3);
mat_pd(:, 1) = (p_ds >= 0.70);
mat_pd(:, 2) = (p_ds >= 0.35 & p_ds < 0.70);
mat_pd(:, 3) = (p_ds <= 0.35);
%mat_pd

v_suc = ((t_up - d_ds) <= 0);
%[d_ds t_up v_suc]

v_all_p = sum(mat_pd)';
v_suc_p = mat_pd' * v_suc;
%v_rate = v_suc_p./ v_all_p;
ret = [v_all_p v_suc_p];

end

