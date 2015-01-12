function [ret] = plan_asap(vec_ds, vec_op)
%   Detailed explanation goes here

b_do = mk_b_od(vec_op, vec_ds)';
b_shift = [zeros(size(b_do, 1), 1) b_do(:, 1:size(b_do, 2) - 1)];
ret = b_do - b_shift;

end

