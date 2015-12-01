function [ret] = vec_mv(v_ds, v_op)
%VEC_MV         Calculate MV vector based on DS and OP
%VEC_MV(v_ds, v_op)
%   v_ds        DS vector
%   v_op        OP vector

x_com = [
    v_ds(:, 1)
    v_op(:, 1)
    ];
x_srt = sort(x_com);
x_srt_shift = [0; x_srt(1:end - 1)];
x_dif = [x_srt(1); x_srt(2:end) - x_srt(1:end - 1)];
ret = [x_srt_shift x_dif];

end

