function [ret] = mk_vec_t1(b_om, vec_mv)
%   Detailed explanation goes here

global V_MDC;

t_mv = vec_mv(:, 2) / V_MDC;
%t1_fin = b_om(:, 1:size(b_om, 2) - 1) * t_mv(1:size(t_mv, 1) - 1, :);
%t1_inf = b_om(:, size(b_om, 2));
%t1_inf(t1_inf == 1) = Inf;
%ret = t1_fin + t1_inf;
ret = b_om * t_mv;

end

