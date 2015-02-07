function [m] = ls_to_m(ls, n)
%LS_TO_M        Convert a schedule list to a schedule matrix
%LS_TO_M(ls)
%   ls          Schedule list

nm = repmat(1:n, size(ls, 1), 1);
lm = repmat(ls, 1, n);
m = (nm == lm).* ones(size(ls, 1), n);

end

