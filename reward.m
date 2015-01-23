function [ret] = reward(v_ds, v_f)
%REWARD         Calculate overall reward
%REWARD(v_ds, v_f)
%   v_ds        DS vector (to get priority values)
%   v_f         Reward vector got from objective function call

ret = sum(v_f.* v_ds(:, 5)) / sum(v_ds(:, 5));

end

