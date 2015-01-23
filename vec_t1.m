function [ret] = vec_t1(b_om, v_mv)
%VEC_T1         Calculate time spent on movement before OP
%VEC_T1(b_om, v_mv)
%   b_om        Matrix B_OM
%   v_mv        MV vector

global V_MDC;

t_mv = v_mv(:, 2) / V_MDC;
ret = b_om * t_mv;

end

