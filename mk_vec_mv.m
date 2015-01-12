function [ret] = mk_vec_mv(vec_ds, vec_op)
%mk_vec_mv      Make a vector of MV
%   Detailed explanation goes here

x_com = [
    vec_ds(:, 1)
    vec_op(:, 1)
    ];
x_srt = sort(x_com);
x_dif = x_srt(2:end) - x_srt(1:end - 1);
ret = [x_srt(1:end - 1) x_dif];

end

