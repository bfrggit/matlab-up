function [m, ls] = plan_asap(v_ds, v_op)
%PLAN_ASAP      Generate a plan using ASAP planning (ASAP upload)
%PLAN_ASAP(v_ds, v_op)
%   v_ds        DS vector
%   v_op        OP vector

b_do = mat_b_od(v_op, v_ds)';
b_shift = [zeros(size(b_do, 1), 1) b_do(:, 1:size(b_do, 2) - 1)];
m = b_do - b_shift;
ls = m_to_ls(m);

end

