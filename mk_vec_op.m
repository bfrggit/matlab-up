function [ret] = mk_vec_op(n_op, dx_mid, er_mu, er_sigma, er_min)
%MK_VEC_OP      Make a vector of OP
%MK_VEC_OP(n_op, dx_mid, er_mu, er_sigma)
%   n_op        Number of OP
%   dx_mid      Mean of distances between DS ~ Uniform dist.
%   er_mu       Mean of estimated uploading rates ~ Normal dist.
%   er_sigma    STDEV of estimated uploading rates

global INF_PSEUDO;

dx = max(round(rand(n_op, 1) * dx_mid * 2), 1);
x = cumsum(dx);
x(end + 1) = INF_PSEUDO;
mu = er_mu / er_min;
sigma = er_sigma / er_min;
er = bsxfun(@max, round(normrnd(mu, sigma, n_op, 1)), 1) * er_min;
er(end + 1) = INF_PSEUDO;
ret = [x er];

end
