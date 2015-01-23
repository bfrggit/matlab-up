function [ret] = rate(v_ds, t_up)
%RATE           Calculate success rates
%RATE(v_ds, t_up)
%   v_ds        DS vector (to get priority values)
%   v_f         Reward vector got from objective function call

global P_DIST;
v_pr = P_DIST(:, 1);

d_ds = v_ds(:, 4);
p_ds = v_ds(:, 5);

% Matrix to identify priority of each DS
mat_pd = zeros(size(v_ds, 1), size(v_pr, 1));
for j = 1:size(v_pr, 1)
    mat_pd(:, j) = (p_ds == v_pr(j));
end
%mat_pd

v_suc = ((t_up - d_ds) <= 0);
%[d_ds t_up v_suc]

v_all_p = sum(mat_pd)';
v_suc_p = mat_pd' * v_suc;
%v_rate = v_suc_p./ v_all_p;
ret = [v_all_p v_suc_p];

end

