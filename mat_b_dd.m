function [ret] = mat_b_dd(s_dd)
%MAT_B_DD       Calculate matrix S_DD (DS at same OP no later than DS)
%MAT_B_DD(s_dd)
%   s_dd        Matrix S_DD

n_ds = size(s_dd, 1);
seq = 1:n_ds;
triangle = (repmat(seq, n_ds, 1) < repmat(seq', 1, n_ds)) + eye(n_ds);
ret = s_dd.* triangle;

end

