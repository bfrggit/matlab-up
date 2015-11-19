function [ret] = mk_vec_op_min(n_op, dx_mid, er_mu, er_sigma, er_min, dx_min)
%MK_VEC_OP_MIN  Make a vector of OP with minimum distance
%MK_VEC_OP_MIN(n_op, dx_mid, er_mu, er_sigma, dx_min)
%   n_op        Number of OP
%   dx_mid      Mean of distances between DS ~ Uniform dist.
%   er_mu       Mean of estimated uploading rates ~ Normal dist.
%   er_sigma    STDEV of estimated uploading rates
%   dx_min      Minimum distance between OP

global INF_PSEUDO;

dx = round(rand(n_op, 1) * (dx_mid - dx_min) * 2 + dx_min);
x = cumsum(dx);
x(end + 1) = INF_PSEUDO;
mu = er_mu / er_min;
sigma = er_sigma / er_min;
er = bsxfun(@max, round(normrnd(mu, sigma, n_op, 1)), 1) * er_min;
er(end + 1) = INF_PSEUDO;
ret = [x er];

end
