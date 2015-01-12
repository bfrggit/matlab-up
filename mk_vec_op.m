function [ret] = mk_vec_op(n_op, dx_mid, er_mu, er_sigma)
% mk_vec_op     Make a vector of OP
%   n_op        Number of OP
%   dx_mid      Mean of distances between DS ~ Uniform dist.
%   er_mu       Mean of estimated uploading rates ~ Normal dist.
%   er_sigma    STDEV of estimated uploading rates

global INF_PSEUDO;

dx = round(rand(n_op, 1) * dx_mid * 2);
x = cumsum(dx);
x(end + 1) = INF_PSEUDO;
er = round(normrnd(er_mu, er_sigma, n_op, 1));
er(end + 1) = Inf;
ret = [x er];

end
