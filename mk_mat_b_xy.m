function [ret] = mk_mat_b_xy(x, y)
%   Detailed explanation goes here

x_rep = repmat(x, 1, size(y));
y_rep = repmat(y', size(x), 1);
ret = (x_rep > y_rep) + zeros(size(x, 1), size(y, 1));

end

