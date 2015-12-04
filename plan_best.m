function [m, ls] = plan_best(v_ds, v_op, t_wait)
%PLAN_BEST      Generate a plan using brute force search, for comparison
%PLAN_BEST(v_ds, v_op)
%   v_ds        DS vector
%   v_op        OP vector
%   t_wait      Time to wait before connection can be established

[m_asap, l_asap] = plan_asap(v_ds, v_op); %#ok<ASGLU>
n_op = size(v_op, 1) - 1;
n_ds = size(v_ds, 1);
l_none = repmat(n_op + 1, n_ds, 1);
l_ind = l_asap;
l_best = l_ind;
r_best = 0;
while sum(l_ind ~= l_none) > 0
    m_ind = ls_to_m(l_ind, n_op + 1);
    r_ind = reward(v_ds, vec_f(v_ds, vec_t_up(v_ds, v_op, m_ind, t_wait)));
    if r_ind > r_best
        l_best = l_ind;
        r_best = r_ind;
    end
    l_ind(n_ds) = l_ind(n_ds) + 1;
    for j = n_ds:-1:1
        if l_ind(j) > n_op + 1
            l_ind(j) = l_asap(j);
            if j > 1
                l_ind(j - 1) = l_ind(j - 1) + 1;
            end
        else break;
        end
    end
end
ls = l_best;
m = ls_to_m(l_best, n_op + 1);

end

