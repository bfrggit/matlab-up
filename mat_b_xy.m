function [ret] = mat_b_xy(x, y)
%MAT_B_XY       Calculate matrix B_XY (Y no later than X)
%               X, Y in {DS, OP, MV}
%MAT_B_XY(x, y)
%   x           X vector
%   y           Y vector

x_rep = repmat(x, 1, size(y, 1));
y_rep = repmat(y', size(x, 1), 1);
ret = (x_rep > y_rep) + zeros(size(x, 1), size(y, 1));

end
